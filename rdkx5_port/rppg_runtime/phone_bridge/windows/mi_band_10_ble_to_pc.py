#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time
import urllib.request

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError


HR_SERVICE_UUID = "0000180d-0000-1000-8000-00805f9b34fb"
HR_MEASUREMENT_UUID = "00002a37-0000-1000-8000-00805f9b34fb"
TARGET_NAME = "xiaomi smart band 10"


def parse_hr_notification(data: bytearray | bytes) -> int | None:
    """Parse standard BLE Heart Rate Measurement value (0x2A37)."""
    if len(data) < 2:
        return None
    flags = data[0]
    if flags & 0x01:
        if len(data) < 3:
            return None
        bpm = int(data[1]) | (int(data[2]) << 8)
    else:
        bpm = int(data[1])
    if 30 <= bpm <= 240:
        return bpm
    return None


def post_to_relay(relay_url: str, bpm: int, source: str) -> None:
    payload = {
        "source": source,
        "bpm": bpm,
        "timestamp": time.time(),
    }
    data = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        relay_url.rstrip("/") + "/reference_hr",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=2.0) as response:
        response.read()


def device_matches(device, adv) -> bool:
    """Check if a BLE device is our target band."""
    name = (device.name or adv.local_name or "").lower()
    return TARGET_NAME in name


async def find_band(timeout: float = 10.0):
    """Scan and return the first Xiaomi Smart Band 10 found."""
    print(f"Scanning for '{TARGET_NAME}' (timeout={timeout:.0f}s)...", flush=True)
    devices = await BleakScanner.discover(timeout=timeout, return_adv=True)
    for address, (dev, adv) in devices.items():
        if device_matches(dev, adv):
            print(f"Found: {dev.name}  [{address}]", flush=True)
            return dev
    print("Band not found.", flush=True)
    return None


async def run(args: argparse.Namespace) -> None:
    # ---- scan-only: list all nearby bands ----
    if args.scan_only:
        print(f"Scanning for BLE devices ({args.scan_seconds:.0f}s)...", flush=True)
        devices = await BleakScanner.discover(timeout=args.scan_seconds, return_adv=True)
        found = []
        for address, (dev, adv) in devices.items():
            if device_matches(dev, adv):
                found.append((address, dev.name or adv.local_name or "", adv.rssi))
        if not found:
            print(f"No '{TARGET_NAME}' found.", flush=True)
            print("Make sure heart-rate broadcast is enabled on the band:", flush=True)
            print("  手环上操作: 上滑 → 设置 → 心率广播 → 开启", flush=True)
            return
        print(f"\nFound {len(found)} device(s):", flush=True)
        for addr, name, rssi in sorted(found, key=lambda x: x[2], reverse=True):
            print(f"  {addr}  {name}  rssi={rssi}", flush=True)
        return

    # ---- check-only: connect and verify HR data ----
    if args.check_only:
        device = await find_band(timeout=args.scan_seconds)
        if device is None:
            print("FAILED: Could not find the band.", flush=True)
            print("Troubleshooting:", flush=True)
            print("  1. Heart-rate broadcast enabled?", flush=True)
            print("     手环上操作: 上滑 → 设置 → 心率广播 → 开启", flush=True)
            print("  2. Band near this computer?", flush=True)
            print("  3. Bluetooth enabled?", flush=True)
            raise SystemExit(1)

        try:
            async with BleakClient(device, timeout=15.0) as client:
                print(f"Connected.", flush=True)
                print(f"Services discovered.", flush=True)
                print(f"\nWaiting for heart-rate data ({args.scan_seconds:.0f}s)...", flush=True)
                found = asyncio.Event()
                result: list[int] = []

                def cb(sender, data):
                    bpm = parse_hr_notification(data)
                    if bpm is not None:
                        result.append(bpm)
                        found.set()

                await client.start_notify(HR_MEASUREMENT_UUID, cb)
                try:
                    await asyncio.wait_for(found.wait(), timeout=args.scan_seconds)
                    print(f"\nCHECK PASSED: received {result[0]} BPM", flush=True)
                except asyncio.TimeoutError:
                    print("No heart-rate data received.", flush=True)
                    print("Make sure heart-rate broadcast is enabled on the band:", flush=True)
                    print("  手环上操作: 上滑 → 设置 → 心率广播 → 开启", flush=True)
                    raise SystemExit(2)
                finally:
                    try:
                        await client.stop_notify(HR_MEASUREMENT_UUID)
                    except Exception:
                        pass
        except BleakError as exc:
            if "bluetooth" in str(exc).lower():
                print("Bluetooth is unavailable on this PC.", flush=True)
                raise SystemExit(3) from exc
            raise
        except Exception as exc:
            print(f"Connection failed: {exc}", flush=True)
            raise SystemExit(1)
        return

    # ---- main loop: connect subscribe relay, auto-reconnect ----
    last_notification_at = 0.0
    last_post_signature: tuple[int, int] | None = None
    last_post_at = 0.0

    def make_handler(relay_url: str):
        nonlocal last_notification_at, last_post_signature, last_post_at

        def handler(sender, data):
            nonlocal last_notification_at, last_post_signature, last_post_at
            bpm = parse_hr_notification(data)
            if bpm is None:
                return
            now = time.time()
            last_notification_at = now
            epoch = int(now / args.repeat_seconds)
            sig = (bpm, epoch)
            if sig == last_post_signature and now - last_post_at < args.repeat_seconds:
                return
            last_post_signature = sig
            last_post_at = now
            try:
                post_to_relay(relay_url, bpm, source=args.source_name)
                print(f"{time.strftime('%H:%M:%S')} bpm={bpm}", flush=True)
            except Exception as exc:
                print(f"POST failed bpm={bpm}: {exc}", flush=True)
        return handler

    while True:
        device = await find_band(timeout=args.scan_seconds)
        if device is None:
            print("Retrying in 5s...", flush=True)
            await asyncio.sleep(5)
            continue

        try:
            async with BleakClient(device, timeout=15.0) as client:
                print(f"Connected. Subscribing to heart-rate...", flush=True)
                handler = make_handler(args.relay_url)
                await client.start_notify(HR_MEASUREMENT_UUID, handler)
                print(f"Listening. Relay -> {args.relay_url}", flush=True)
                while True:
                    await asyncio.sleep(10)
                    idle = time.time() - last_notification_at
                    if idle > 30:
                        print(f"No HR for {idle:.0f}s, reconnecting...", flush=True)
                        break
        except asyncio.CancelledError:
            break
        except Exception as exc:
            print(f"Error: {exc}. Reconnecting in 5s...", flush=True)
            await asyncio.sleep(5)


def main() -> int:
    parser = argparse.ArgumentParser(description="Xiaomi Smart Band 10 BLE Heart Rate Bridge")
    parser.add_argument("--relay-url", default="http://127.0.0.1:8090")
    parser.add_argument("--repeat-seconds", type=float, default=2.0)
    parser.add_argument("--scan-seconds", type=float, default=10.0)
    parser.add_argument("--source-name", default="mi_band_10_ble")
    parser.add_argument("--scan-only", action="store_true", help="Scan for nearby devices")
    parser.add_argument("--check-only", action="store_true", help="Connect and verify HR works")
    args = parser.parse_args()

    try:
        asyncio.run(run(args))
    except (KeyboardInterrupt, SystemExit):
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
