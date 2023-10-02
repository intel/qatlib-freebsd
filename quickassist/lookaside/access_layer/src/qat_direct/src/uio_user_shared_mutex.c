/***************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2023 Intel Corporation. All rights reserved.
 * 
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 * 
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 * 
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *   The full GNU General Public License is included in this distribution
 *   in the file called LICENSE.GPL.
 * 
 *   Contact Information:
 *   Intel Corporation
 * 
 *   BSD LICENSE
 * 
 *   Copyright(c) 2007-2023 Intel Corporation. All rights reserved.
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
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
