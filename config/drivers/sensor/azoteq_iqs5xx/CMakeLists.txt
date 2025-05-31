# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_include_directories(.)

message(AZO LALALAL="${CONFIG_IQS5XX}")

zephyr_library_sources(azoteq_iqs5xx.c)
