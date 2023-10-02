# Intel&reg; QuickAssist Technology Library (QATlib)

## Table of Contents

- [Intel® QuickAssist Technology Library (QATlib)](#intel-quickassist-technology-library-qatlib)
  - [Table of Contents](#table-of-contents)
  - [Revision History](#revision-history)
  - [Overview](#overview)
  - [Features](#features)
  - [Deprecated Features \& Planned Deprecations](#deprecated-features--planned-deprecations)
  - [Setup](#setup)
  - [Supported Devices](#supported-devices)
  - [Limitations](#limitations)
  - [Environmental Assumptions](#environmental-assumptions)
  - [Examples](#examples)
  - [Open Issues](#open-issues)
  - [Resolved Issues](#resolved-issues)
  - [Licensing](#licensing)
  - [Legal](#legal)
  - [Terminology](#terminology)

## Revision History

| Date      |     Doc Revision      | Version |   Details |
|----------|:-------------:|------:|:------|
| September 2023 | 001 | 23.09 | - Initial Release |

## Overview
Intel(R) QuickAssist Technology (Intel(R) QAT) provides
hardware acceleration for offloading security, authentication and
compression services from the CPU, thus significantly increasing
the performance and efficiency of standard platform solutions.

Its services include symmetric encryption and authentication,
asymmetric encryption, digital signatures, RSA, DH and ECC, and
lossless data compression.

This package provides user space libraries that allow access to
Intel(R) QuickAssist devices and expose the Intel(R) QuickAssist APIs and
sample codes.

## Features

The following services are available in QATlib via the QuickAssist API:
* Symmetric (Bulk) Cryptography
  * Ciphers (AES-ECB, AES-CBC, AES-CTR (no partials support),
    AES-XTS (no partials support), AES-GCM, AES-CCM (192/256)
  * Message digest/hash (SHA1, SHA2 (224/256/384/512),
    SHA3 (224/256/384/512) (no partials support) and
    authentication (AES-CBC-MAC, AES-XCBC-MAC)
  * Algorithm chaining (one cipher and one hash in a single operation)
  * Authenticated encryption (CCM-128 (no partials support),
    GCM (128/192/256) (no partials support), GMAC (no partials support)
    and ChaCha20-Poly1305)
* KeyGen
  * TLS1.2
  * TLS1.3
  * HKDF
  * MGF1
* Asymmetric (Public Key) Cryptography
  * Modular exponentiation and modular inversion up to 8192 bits
  * Diffie-Hellman (DH) key generation phase 1 and 2 up to 8192 bits
  * RSA key generation, encryption/decryption and digital signature
    generation/verification up to 8192 bits
  * DSA parameter generation and digital signature generation/verification
  * Elliptic Curve Cryptography: ECDSA, ECDHE, Edwards Montgomery curves
  * Generic point multiply
* Compression
  * Deflate
  * Compress and Verify (CnV)
  * Compress and Verify and Recover (CnVnR)
  * End-to-end (E2E) integrity check

This package includes:
* libqat: user space library for QAT devices exposed via the UIO kernel driver
* libusdm: user space library and kernel space driver for memory management
* Sample codes: applications to demo usage of the libs

## Deprecated Features & Planned Deprecations
* The following configuration option will be deprecated after 2023:
  * `--enable-legacy-lib-names`

## Setup
Please refer to [INSTALL](INSTALL) for details on installing the library.

## Supported Devices
* 4xxx (QAT gen 4 devices)

Earlier generations of QAT devices (e.g. c62x, dh895xxcc, etc.) are not
supported.

## Limitations
* If an error occurs on the host driver (Heartbeat, Uncorrectable error) it
  will not be communicated to the library.

The following features are not currently supported:
* Dynamic instances
* lz4/lz4s compression
* Intel® Key Protection Technology (KPT)
* Event driven polling
* Maximum 16 processes per end point
* integrityCrcCheck for Compression direction requests


## Environmental Assumptions

The following assumptions are made concerning the deployment environment:
* Users within the same processing domain must be trusted.
* The Intel® QAT device should not be exposed (via the "user space direct"
  deployment model) to untrusted users.
* DRAM is considered to be inside the trust boundary. The typical memory
  protection schemes provided by the Intel architecture processor and memory
  controller, and by the operating system, prevent unauthorized access to these
  memory regions.
* A QuickAssist kernel driver for the supported device is installed, which has
  discovered and initialised the device. This driver is included in
  the FreeBSD kernel, see [INSTALL](INSTALL) for information about which kernel
  to use.
* The library can be used only by root user.

## Examples
Example applications that showcase usage of the QAT APIs are included in the
package (quickassist/lookaside/access_layer/src/sample_code).
Please refer to [Intel® QuickAssist Technology API Programmer's Guide](https://www.intel.com/content/www/us/en/content-details/709196/intel-quickassist-technology-api-programmer-s-guide.html).

## Open Issues
Known issues relating to the Intel® QAT software are described
in this section.

Issue titles follow the pattern:

    <Component> [Stepping] -  Description of issue
where: \<Component\> is one of the following:
* CY - Cryptographic
* DC - Compression
* EP - Endpoint
* GEN - General
* SYM DP - Symmetric Cryptography on Data Plane
* SR-IOV - Single Root I/O Virtualization
* FW - Firmware
* PERF - Performance

[Stepping] is an optional qualifier that identifies if the errata applies to a specific device stepping

| Issue ID | Description |
|-------------|------------|
| QATE-3241  | [CY - cpaCySymPerformOp when used with parameter checking may reveal the amount of padding.](#qate-3241) |
| QATE-41707 | [CY - Incorrect digest returned when performing a plain hash operation on input data of size 4GB or larger.](#qate-41707) |

## QATE-3241
| Title      |       CY - cpaCySymPerformOp when used with parameter checking may reveal the amount of padding.        |
|----------|:-------------
| Reference # | QATE-3241 |
| Description | When Performing a CBC Decryption as a chained request using cpaCySymPerformOp it is necessary to pass a length of the data to MAC (messageLenToHashInBytes). With ICP_PARAM_CHECK enabled, this checks the length of data to MAC is valid and, if not, it aborts the whole operation and outputs an error on stderr. |
| Implication | The length of the data to MAC is based on the amount of padding. This should remain private and not be revealed. The issue is not observed when the length is checked in constant time before passing the value to the API. This is done by OpenSSL. |
| Resolution | 1. Build without ICP_PARAM_CHECK, but this opens the risk of buffer overrun. <BR> 2. Validate the length before using the API. |
| Affected OS | FreeBSD |
| Driver/Module | CPM-IA - Crypto |

## QATE-41707

| Title      |         CY -  Incorrect digest returned when performing a plain hash operation on input data of size 4GB or larger.      |
|----------|:-------------
| Reference # | QATE-41707 |
| Description | When performing a plain hash operation on input data size of 4GB or larger, incorrect digest is returned. |
| Implication | Incorrect digest is returned from a plane hash operation. |
| Resolution | There is no fix available. |
| Affected OS | FreeBSD |
| Driver/Module | CPM-IA - Crypto |

## Resolved Issues
Resolved issues relating to the Intel® QAT software are described
in this section.

 ## Licensing
* This product is released under the BSD-3-Clause.

## Legal

Intel, Intel Atom, and Xeon are trademarks of
Intel Corporation in the U.S. and/or other countries.

*Other names and brands may be claimed as the property of others.

Copyright &copy; 2016-2022, Intel Corporation. All rights reserved.

## Terminology

| Term   |      Description      |
|----------|:-------------:|
| API | Application Programming Interface |
| BIOS | Basic Input/Output System |
| BSD | Berkeley Standard Distribution |
| CY | Cryptographic |
| CnV | Compress and Verify |
| CnVnR | Compress and Verify and Recover |
| DC | Compression |
| DMA | Direct Memory Access |
| EFI | Extensible Firmware Interface |
| FW | Firmware |
| GPL | General Public License |
| HKDF | HMAC-based Extract-and-Expand Key Derivation Function |
| Intel® QAT | Intel® QuickAssist Technology |
| OS | Operating System |
| SR-IOV | Single-root Input/Output Virtualization |
| TLS | Transport Layer Security |
| VFs | Virtual Functions |
