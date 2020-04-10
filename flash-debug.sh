#!/usr/bin/env bash

set -ex

kind="debug"
name="hello-bluefruit"

cargo build

arm-none-eabi-objcopy -O binary "target/thumbv6m-none-eabi/${kind}/${name}" "target/thumbv6m-none-eabi/${kind}/${name}.bin"

bossac -e -w -v -R --offset=0x2000 "target/thumbv6m-none-eabi/${kind}/${name}.bin"
