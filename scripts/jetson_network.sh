#!/usr/bin/env bash
# 在宿主机运行：为直连 Jetson 保存专用静态配置，不接管默认路由。
set -euo pipefail

usage() {
  cat <<'EOF'
用法：bash scripts/jetson_network.sh [up|down|status] [网卡]

默认：up enp0s20f0u2u2c2
本机：172.16.0.20/24；Jetson：172.16.0.10
up 保存并启用 NetworkManager 专用连接，然后通过指定网卡 ping Jetson。
不配置网关/DNS，不自动连接；重复 up 可复用配置。
down 仅断开本脚本的连接，保留配置供下次使用。
如 NetworkManager 提示权限不足，请在宿主机用 sudo 运行。
EOF
}

action=${1:-up}
interface=${2:-enp0s20f0u2u2c2}
profile="casbot-jetson-${interface}"
address=172.16.0.20/24
peer=172.16.0.10

if [[ $action == -h || $action == --help ]]; then usage; exit 0; fi
if (($# > 2)); then usage >&2; exit 2; fi
case "$action" in up|down|status) ;; *) usage >&2; exit 2 ;; esac

case "$action" in
  up)
    ip link show dev "$interface" >/dev/null
    active=$(nmcli -g GENERAL.CONNECTION device show "$interface")
    if [[ -n $active && $active != -- && $active != "$profile" ]]; then
      printf '网卡 %s 已由连接 %s 使用；请先明确断开该连接。\n' "$interface" "$active" >&2
      exit 1
    fi
    if nmcli connection show id "$profile" >/dev/null 2>&1; then
      bound=$(nmcli -g connection.interface-name connection show id "$profile")
      [[ $bound == "$interface" ]] || { echo '已有同名配置绑定其它网卡，未修改。' >&2; exit 1; }
      nmcli connection modify id "$profile" \
        connection.autoconnect no ipv4.method manual ipv4.addresses "$address" \
        ipv4.gateway '' ipv4.routes '' ipv4.never-default yes \
        ipv4.dns '' ipv4.ignore-auto-dns yes ipv6.method disabled
    else
      nmcli connection add type ethernet ifname "$interface" con-name "$profile" \
        connection.autoconnect no ipv4.method manual ipv4.addresses "$address" \
        ipv4.never-default yes ipv4.ignore-auto-dns yes ipv6.method disabled
    fi
    nmcli --wait 15 connection up id "$profile" ifname "$interface"
    ip -4 addr show dev "$interface"
    ip -4 route get "$peer" oif "$interface"
    ping -n -I "$interface" -c 4 -W 2 "$peer"
    ;;
  down)
    nmcli --wait 10 connection down id "$profile"
    ;;
  status)
    ip -br addr show dev "$interface"
    nmcli -f GENERAL.STATE,GENERAL.CONNECTION,IP4 device show "$interface"
    ;;
esac
