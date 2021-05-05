/*
 * Copyright (c) 2019, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef WISUN_CUSTOM_CERTIFICATES_H_
#define WISUN_CUSTOM_CERTIFICATES_H_

const uint8_t WISUN_ROOT_CERTIFICATE[] = {
    "-----BEGIN CERTIFICATE-----\r\n"
    "MIICmDCCAj2gAwIBAgIJAPXEa83exaPFMAoGCCqGSM49BAMCMIGfMQswCQYDVQQG\r\n"
    "EwJGUjELMAkGA1UECAwCRlIxDzANBgNVBAcMBlJlbm5lczEcMBoGA1UECgwTU2ls\r\n"
    "aWNvbiBMYWJzIEZyYW5jZTEMMAoGA1UECwwDUiZEMRowGAYDVQQDDBFXaS1TVU4g\r\n"
    "VGVzdGluZyBDQTEqMCgGCSqGSIb3DQEJARYbdHVvbWFzLm1hYXR0YW5lbkBzaWxh\r\n"
    "YnMuY29tMB4XDTIwMDExNDE0NDEwNloXDTI1MDExMjE0NDEwNlowgZ8xCzAJBgNV\r\n"
    "BAYTAkZSMQswCQYDVQQIDAJGUjEPMA0GA1UEBwwGUmVubmVzMRwwGgYDVQQKDBNT\r\n"
    "aWxpY29uIExhYnMgRnJhbmNlMQwwCgYDVQQLDANSJkQxGjAYBgNVBAMMEVdpLVNV\r\n"
    "TiBUZXN0aW5nIENBMSowKAYJKoZIhvcNAQkBFht0dW9tYXMubWFhdHRhbmVuQHNp\r\n"
    "bGFicy5jb20wWTATBgcqhkjOPQIBBggqhkjOPQMBBwNCAAQKqkm1zos92BwD6jxa\r\n"
    "g82zozNwbTw6Y1d9APcH6RCWzaoTYXu13zewktPYPU4AocrkK2jJdt7cxipbxiSx\r\n"
    "dRQro2AwXjAPBgNVHRMBAf8EBTADAQH/MAsGA1UdDwQEAwIBBjAdBgNVHQ4EFgQU\r\n"
    "quZSRE1EjJDyqY10hUWsHWjwAtEwHwYDVR0jBBgwFoAUquZSRE1EjJDyqY10hUWs\r\n"
    "HWjwAtEwCgYIKoZIzj0EAwIDSQAwRgIhALkJ+4FYq+1vdobQrQI5LrCkkNPe/NeS\r\n"
    "v7vDalV6UHqSAiEA8ncGWs1haoL/UomMo5bXtS+ov8wV6ibwp5vMkf0ckio=\r\n"
    "-----END CERTIFICATE-----"
};

const uint8_t WISUN_SERVER_CERTIFICATE[] = {
    "-----BEGIN CERTIFICATE-----\r\n"
    "MIIClDCCAjugAwIBAgIJAND+U5bQNkNiMAoGCCqGSM49BAMCMIGfMQswCQYDVQQG\r\n"
    "EwJGUjELMAkGA1UECAwCRlIxDzANBgNVBAcMBlJlbm5lczEcMBoGA1UECgwTU2ls\r\n"
    "aWNvbiBMYWJzIEZyYW5jZTEMMAoGA1UECwwDUiZEMRowGAYDVQQDDBFXaS1TVU4g\r\n"
    "VGVzdGluZyBDQTEqMCgGCSqGSIb3DQEJARYbdHVvbWFzLm1hYXR0YW5lbkBzaWxh\r\n"
    "YnMuY29tMB4XDTIwMDExNTE2MzMzNFoXDTI1MDExMzE2MzMzNFowgaMxCzAJBgNV\r\n"
    "BAYTAkZSMQswCQYDVQQIDAJGUjEPMA0GA1UEBwwGUmVubmVzMRwwGgYDVQQKDBNT\r\n"
    "aWxpY29uIExhYnMgRnJhbmNlMQwwCgYDVQQLDANSJkQxHjAcBgNVBAMMFVdpLVNV\r\n"
    "TiBUZXN0aW5nIFNlcnZlcjEqMCgGCSqGSIb3DQEJARYbdHVvbWFzLm1hYXR0YW5l\r\n"
    "bkBzaWxhYnMuY29tMFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAEH5DaEIN1AGBe\r\n"
    "Q4s1K5IBvHn4VPHVSKUliZbYg2QBCWlLfD8IL14zto05uYfN7ffH8M2fyM2BiaxN\r\n"
    "BoRonczc+KNaMFgwCQYDVR0TBAIwADALBgNVHQ8EBAMCBeAwHQYDVR0OBBYEFDTl\r\n"
    "6IbtAgU0ogpxJF9rBhbWnEC9MB8GA1UdIwQYMBaAFKrmUkRNRIyQ8qmNdIVFrB1o\r\n"
    "8ALRMAoGCCqGSM49BAMCA0cAMEQCIHKDn9kL5koqbcrW5gQrB//7YlgOm9iu5hju\r\n"
    "yU5ruOFMAiBoOwjBho3mBRFjInwW6/CmlZ/XwMSjkVzUcwpeR1dozA==\r\n"
    "-----END CERTIFICATE-----"
};

const uint8_t WISUN_SERVER_KEY[] = {
    "-----BEGIN EC PRIVATE KEY-----\r\n"
    "MHcCAQEEILRF7+wGLZ7ZVqEZBhovNC2ml5ZTzR7vpjRd3WnkBnXRoAoGCCqGSM49\r\n"
    "AwEHoUQDQgAEH5DaEIN1AGBeQ4s1K5IBvHn4VPHVSKUliZbYg2QBCWlLfD8IL14z\r\n"
    "to05uYfN7ffH8M2fyM2BiaxNBoRonczc+A==\r\n"
    "-----END EC PRIVATE KEY-----"
};

const uint8_t WISUN_CLIENT_CERTIFICATE[] = {
    "-----BEGIN CERTIFICATE-----\r\n"
    "MIIC4zCCAomgAwIBAgIJAND+U5bQNkNkMAoGCCqGSM49BAMCMIGfMQswCQYDVQQG\r\n"
    "EwJGUjELMAkGA1UECAwCRlIxDzANBgNVBAcMBlJlbm5lczEcMBoGA1UECgwTU2ls\r\n"
    "aWNvbiBMYWJzIEZyYW5jZTEMMAoGA1UECwwDUiZEMRowGAYDVQQDDBFXaS1TVU4g\r\n"
    "VGVzdGluZyBDQTEqMCgGCSqGSIb3DQEJARYbdHVvbWFzLm1hYXR0YW5lbkBzaWxh\r\n"
    "YnMuY29tMB4XDTIxMDIxOTE1MDQxMloXDTI2MDIxODE1MDQxMlowgaMxCzAJBgNV\r\n"
    "BAYTAkZSMQswCQYDVQQIDAJGUjEPMA0GA1UEBwwGUmVubmVzMRwwGgYDVQQKDBNT\r\n"
    "aWxpY29uIExhYnMgRnJhbmNlMQwwCgYDVQQLDANSJkQxHjAcBgNVBAMMFVdpLVNV\r\n"
    "TiBUZXN0aW5nIENsaWVudDEqMCgGCSqGSIb3DQEJARYbdHVvbWFzLm1hYXR0YW5l\r\n"
    "bkBzaWxhYnMuY29tMFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAEREjG2cnUrYPI\r\n"
    "n9Ul8IwQnqX4BEw6hRnyVeVYtGayfDDVdeS6PTtiq9olKB85oe+KBhkoe1xKlLbb\r\n"
    "TgoExIuHJKOBpzCBpDAJBgNVHRMEAjAAMAsGA1UdDwQEAwIF4DAeBgNVHSUEFzAV\r\n"
    "BgkrBgEEAYLkJQEGCCsGAQUFBwMCMCoGA1UdEQQjMCGgHwYIKwYBBQUHCASgEzAR\r\n"
    "BgcrBgEEASoBBAYxMjM0NTYwHQYDVR0OBBYEFPJ/DhD/+MafDAuRPI2mmG+5uJYK\r\n"
    "MB8GA1UdIwQYMBaAFKrmUkRNRIyQ8qmNdIVFrB1o8ALRMAoGCCqGSM49BAMCA0gA\r\n"
    "MEUCIEg132X2Sy6UjiOETS0tNA9JLGEa9J09g9pnptUP6N0vAiEAx4gAF92VtAch\r\n"
    "QNsut8oc+3R25a4imdSGuHMyxWT/RJU=\r\n"
    "-----END CERTIFICATE-----"
};

const uint8_t WISUN_CLIENT_KEY[] = {
    "-----BEGIN EC PRIVATE KEY-----\r\n"
    "MHcCAQEEINEvBfH5Pg7PdpZXENMIAafWCp3a0cX3PGPQtNsQrBhuoAoGCCqGSM49\r\n"
    "AwEHoUQDQgAEREjG2cnUrYPIn9Ul8IwQnqX4BEw6hRnyVeVYtGayfDDVdeS6PTti\r\n"
    "q9olKB85oe+KBhkoe1xKlLbbTgoExIuHJA==\r\n"
    "-----END EC PRIVATE KEY-----"
};

#endif /* WISUN_CUSTOM_CERTIFICATES_H_ */
