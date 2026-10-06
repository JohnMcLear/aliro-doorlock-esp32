#!/usr/bin/env python3
"""Generate a matching set of Aliro TEST keys for the ring applet test mode.

Prints two things:

  1. the serial console commands that load the reader side into a board
     running the c6-st25r300-wifi-ringtest firmware, and
  2. the values the credential side (for example a Java Card applet) needs.

Every key printed here is a TEST key. Anyone who has this output can open a
reader configured with it. Never load these into a reader on a real door, and
never reuse them for anything else.

Usage:
  tools/ring_test_keys.py                      # one reader, one endpoint
  tools/ring_test_keys.py --endpoints 3        # several test credentials
  tools/ring_test_keys.py --out keys.json      # also save everything as JSON

Needs the Python 'cryptography' package (pip install cryptography).
"""

import argparse
import json
import os
import sys

try:
    from cryptography.hazmat.primitives import serialization
    from cryptography.hazmat.primitives.asymmetric import ec
except ImportError:  # pragma: no cover
    sys.exit("error: needs the 'cryptography' package: pip install cryptography")

BANNER = "=" * 78
WARNING = "ALIRO TEST KEYS. NOT FOR A REAL DOOR. Anyone with this output can open the reader."


def p256_keypair():
    key = ec.generate_private_key(ec.SECP256R1())
    priv = key.private_numbers().private_value.to_bytes(32, "big")
    pub = key.public_key().public_bytes(
        serialization.Encoding.X962, serialization.PublicFormat.UncompressedPoint
    )
    pub_pem = key.public_key().public_bytes(
        serialization.Encoding.PEM, serialization.PublicFormat.SubjectPublicKeyInfo
    ).decode()
    return priv, pub, pub_pem


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--endpoints", type=int, default=1, help="number of test credentials (1..8)")
    parser.add_argument("--out", help="also write the keys to this JSON file")
    args = parser.parse_args()
    if not 1 <= args.endpoints <= 8:
        parser.error("--endpoints must be 1..8 (the reader holds 8)")

    reader_priv, reader_pub, reader_pub_pem = p256_keypair()
    group_id = os.urandom(16)
    endpoints = []
    for _ in range(args.endpoints):
        priv, pub, pem = p256_keypair()
        endpoints.append({"private_key": priv.hex(), "public_key": pub.hex(), "public_key_pem": pem})

    print(BANNER)
    print(WARNING)
    print(BANNER)
    print()
    print("1. READER: paste these lines into the board's USB serial console")
    print("   (115200 baud is fine; the USB Serial/JTAG port ignores it):")
    print()
    print(f"ring reader {reader_priv.hex()} {group_id.hex()}")
    for ep in endpoints:
        print(f"ring endpoint add {ep['public_key']}")
    print("ring trust list")
    print("ring show")
    print()
    print("   'ring show' then prints the reader group sub-identifier, which the")
    print("   reader generates itself and keeps in NVS. Some credential")
    print("   implementations want it as the second half of the reader identifier.")
    print()
    print("2. CREDENTIAL (applet) side:")
    print()
    print(f"   Reader public key (65 bytes, uncompressed P-256, verifies the reader's AUTH1 signature):")
    print(f"     {reader_pub.hex()}")
    print(f"   Reader group identifier (16 bytes):")
    print(f"     {group_id.hex()}")
    for index, ep in enumerate(endpoints):
        print(f"   Endpoint {index} private key (32 bytes, P-256, signs the credential's AUTH1 response):")
        print(f"     {ep['private_key']}")
        print(f"   Endpoint {index} public key (65 bytes, uncompressed):")
        print(f"     {ep['public_key']}")
    print()
    print("   Reader public key as PEM, if your tooling prefers it:")
    print(reader_pub_pem.rstrip())
    print()
    print(BANNER)
    print(WARNING)
    print(BANNER)

    if args.out:
        document = {
            "warning": WARNING,
            "reader": {
                "private_key": reader_priv.hex(),
                "public_key": reader_pub.hex(),
                "public_key_pem": reader_pub_pem,
                "group_identifier": group_id.hex(),
            },
            "endpoints": endpoints,
            "serial_commands": [f"ring reader {reader_priv.hex()} {group_id.hex()}"]
            + [f"ring endpoint add {ep['public_key']}" for ep in endpoints]
            + ["ring trust list", "ring show"],
        }
        with open(args.out, "w", encoding="utf-8") as handle:
            json.dump(document, handle, indent=2)
            handle.write("\n")
        print(f"\nwritten to {args.out}")


if __name__ == "__main__":
    main()
