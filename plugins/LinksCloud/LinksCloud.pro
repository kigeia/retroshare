!include("../Common/retroshare_plugin.pri"): error("Could not include file ../Common/retroshare_plugin.pri")

CONFIG *= qt uic debug resources

SOURCES = p3ranking.cc LinksDialog.cpp rsrankitems.cc AddLinksDialog.cpp LinksCloudPlugin.cpp
HEADERS = rsrank.h p3ranking.h LinksDialog.h rsrankitems.h AddLinksDialog.h LinksCloudPlugin.h
FORMS   = LinksDialog.ui AddLinksDialog.ui

TARGET = LinksCloud

RESOURCES = images.qrc
