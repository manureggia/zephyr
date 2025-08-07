# Copyright 2024 Université Gustave Eiffel
# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_ARM_MMU mmu_regions.c)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm64/scripts/linker.ld CACHE INTERNAL "")
# Compila il file soc.c quando questo SoC è attivo
zephyr_sources(soc.c)
zephyr_include_directories(include)
