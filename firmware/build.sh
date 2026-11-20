#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace"

if [[ ! -x .venv/bin/west || ! -f zephyr/zephyr-env.sh || ! -d .west ]]; then
    printf '%s\n' '请先按 firmware/README.md 初始化本地 Zephyr 工作区。' >&2
    exit 1
fi

app=trical_bridge
output=trical_bridge
case "${1:-}" in
    --rate-test)
        app=tests/qas_rate
        output=qas_rate
        shift
        ;;
    --example)
        case "${2:-}" in
            blinky|usb_cdc)
                app="examples/$2"
                output="$2"
                shift 2
                ;;
            *)
                printf '%s\n' '用法：bash firmware/build.sh --example {blinky|usb_cdc} [west build 参数]' >&2
                exit 2
                ;;
        esac
        ;;
esac

# Keep this workspace independent of any globally exported/shared Zephyr tree.
export ZEPHYR_BASE="$workspace/zephyr"
exec .venv/bin/west build \
    -b promicro_nrf52840/nrf52840/uf2 \
    -d "$workspace/build/$output" \
    "$workspace/$app" "$@"
