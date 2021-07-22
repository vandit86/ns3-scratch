## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

import wutils
bld = wutils.bld

all_modules = [mod[len("ns3-"):] for mod in bld.env['NS3_ENABLED_MODULES'] + bld.env['NS3_ENABLED_CONTRIBUTED_MODULES']]

def build(bld):
    obj = bld.create_ns3_program('mp-fd-wifi-lte',all_modules)
    obj.source = 'mp-fd-wifi-lte.cc'