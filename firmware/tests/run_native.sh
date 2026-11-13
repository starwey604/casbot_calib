#!/usr/bin/env bash
set -euo pipefail
firmware="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$firmware/build/native-tests"
cc -std=c11 -Wall -Wextra -Werror -g -fsanitize=address,undefined \
    -I"$firmware/trical_bridge/lib" \
    "$firmware/trical_bridge/lib/qas_protocol.c" \
    "$firmware/trical_bridge/lib/rate_core.c" \
    "$firmware/tests/native/test_core.c" \
    -o "$firmware/build/native-tests/test_core"
"$firmware/build/native-tests/test_core"
cc -std=c11 -D_POSIX_C_SOURCE=200809L -Wall -Wextra -Werror -g -fsanitize=address,undefined \
    -I"$firmware/tests/native/fakes" -I"$firmware/trical_bridge/lib" \
    "$firmware/trical_bridge/lib/qas_client.c" \
    "$firmware/trical_bridge/lib/qas_protocol.c" \
    "$firmware/trical_bridge/lib/rate_core.c" \
    "$firmware/tests/native/test_client.c" \
    -o "$firmware/build/native-tests/test_client"
"$firmware/build/native-tests/test_client" > "$firmware/build/native-tests/usb-fixture.jsonl"
cc -std=c11 -Wall -Wextra -Werror -g -fsanitize=address,undefined \
    -I"$firmware/tests/native/fakes" -I"$firmware/trical_bridge/lib" \
    "$firmware/trical_bridge/lib/usb_transport.c" "$firmware/tests/native/test_usb.c" \
    -o "$firmware/build/native-tests/test_usb"
"$firmware/build/native-tests/test_usb"
"$firmware/.venv/bin/python" -m unittest discover -s "$firmware/tests/native" -p 'test_*.py' -v
