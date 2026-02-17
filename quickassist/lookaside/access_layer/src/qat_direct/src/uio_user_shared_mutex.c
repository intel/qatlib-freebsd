/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#ifndef ICP_WITHOUT_THREAD
#include <pthread.h>
#endif
#include "icp_platform.h"

#define UIO_MUTEX_TAG 12345
#define UIO_MUTEX_PREFIX "/var/tmp/shm_key_%s"
#define UIO_MUTEX_PERMISSION 0x1b6
#define UIO_MUTEX_NAME_MAX_LENGHT 128
#ifndef EOWNERDEAD
#define EOWNERDEAD 96
#endif

#define GET_TAG(mem) ((int *)(((char *)mem) + sizeof(pthread_mutex_t)))
#ifndef ICP_WITHOUT_THREAD
static int uio_shared_memory_mutex_init(pthread_mutex_t *mutex)
{
    int err;
    pthread_mutexattr_t mutexattribute;

    err = pthread_mutex_init(mutex, NULL);

    if (err == EBUSY || err == 0)
    {
        return 0; /*Treat busy as success*/
    }
    pthread_mutexattr_destroy(&mutexattribute);
    return errno;
}

int uio_shared_mutex_create(const char *name, pthread_mutex_t **mutex)
{
    int shmid;
    int created;
    int *tag;
    FILE *file;
    void *shmem;
    char full_file_name[UIO_MUTEX_NAME_MAX_LENGHT] = { 0 };
    key_t semkey;

    snprintf(full_file_name, UIO_MUTEX_NAME_MAX_LENGHT, UIO_MUTEX_PREFIX, name);
    /*create the file if needed*/
    file = fopen(full_file_name, "a");
    if (!file)
    {
        return errno;
    }
    fclose(file);

    semkey = ftok(full_file_name, sizeof(full_file_name));
    if (semkey == -1)
    {
        perror("ftok failed\n");

        return errno;
    }

    shmid = shmget(semkey,
                   sizeof(pthread_mutex_t) + sizeof(int),
                   IPC_CREAT | IPC_EXCL | UIO_MUTEX_PERMISSION);
    if (shmid == -1)
    {
        if (errno == EEXIST)
        {
            /*we are not the first process*/
            shmid = shmget(semkey,
                           sizeof(pthread_mutex_t) + sizeof(int),
                           UIO_MUTEX_PERMISSION);
            if (shmid == -1)
            {
                return errno; /*could not find it neither*/
            }
            else
                created = 0; /*shared memory retrieved*/
        }
        else
        {

            return errno; /*cannot create ,and does not exist*/
        }
    }
    else
    {
        created = 1; /*shared memory created succesfully*/
    }

    shmem = shmat(shmid, NULL, 0);
    if (shmem == (void *)-1)
    {

        return errno;
    }
    tag = GET_TAG(shmem);

    if (created || (*tag != UIO_MUTEX_TAG))
    {

        if (uio_shared_memory_mutex_init((pthread_mutex_t *)shmem))
        {
            ADF_ERROR("uio_shared_mutex_create fail errno %d \n", errno);
            return -ENODEV;
        }

        file = fopen(full_file_name, "a");
        if (!file)
        {

            return errno;
        }

        fprintf(file, "key:0x%x shmid:%i\r\n", (unsigned int)semkey, shmid);
        fclose(file);
        *tag = UIO_MUTEX_TAG;
    }
    *mutex = (pthread_mutex_t *)shmem;

    return 0;
}
int uio_shared_mutex_lock(pthread_mutex_t *mutex)
{
    int err;
    err = pthread_mutex_lock(mutex);

    if (err == EOWNERDEAD)
    {
        pthread_mutex_unlock(mutex);
    }
    if (err == 11) // deadlock avoided
        err = 0;
    return err;
}

int uio_shared_mutex_unlock(pthread_mutex_t *mutex)
{
    return pthread_mutex_unlock(mutex);
}
int uio_shared_mutex_destroy(pthread_mutex_t *mutex)
{
    return pthread_mutex_destroy(mutex);
}
#endif
