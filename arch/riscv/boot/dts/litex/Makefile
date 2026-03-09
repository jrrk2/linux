# SPDX-License-Identifier: GPL-2.0
dtb-$(CONFIG_BUILTIN_DTB) += sonata.dtb

# Decode pre-built DTB from base64 (avoids dtc dependency)
$(obj)/sonata.dtb: $(src)/sonata.dtb.b64
	base64 -d < $< > $@
