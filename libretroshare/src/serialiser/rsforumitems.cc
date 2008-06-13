
/*
 * libretroshare/src/serialiser: rsforumitems.cc
 *
 * RetroShare Serialiser.
 *
 * Copyright 2007-2008 by Robert Fernie.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License Version 2 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA.
 *
 * Please report all bugs and problems to "retroshare@lunamutt.com".
 *
 */

#include "serialiser/rsforumitems.h"

#include "serialiser/rsbaseserial.h"
#include "serialiser/rstlvbase.h"

#define RSSERIAL_DEBUG 1
#include <iostream>

/*************************************************************************/

void 	RsForumMsg::clear()
{
	RsDistribMsg::clear();

	srcId.clear();
	title.clear();
	msg.clear();
}

std::ostream &RsForumMsg::print(std::ostream &out, uint16_t indent)
{
        printRsItemBase(out, "RsForumMsg", indent);
	uint16_t int_Indent = indent + 2;

	RsDistribMsg::print(out, int_Indent);

        printIndent(out, int_Indent);
        out << "srcId: " << srcId << std::endl;

        printIndent(out, int_Indent);

	std::string cnv_title(title.begin(), title.end());
        out << "title:  " << cnv_title  << std::endl;

        printIndent(out, int_Indent);

	std::string cnv_msg(msg.begin(), msg.end());
        out << "msg:  " << cnv_msg  << std::endl;

        printRsItemEnd(out, "RsForumMsg", indent);
        return out;
}


/*************************************************************************/
/*************************************************************************/

uint32_t    RsForumSerialiser::sizeMsg(RsForumMsg *item)
{
	uint32_t s = 8; /* header */
	/* RsDistribMsg stuff */
	s += GetTlvStringSize(item->grpId);
	s += GetTlvStringSize(item->parentId);
	s += GetTlvStringSize(item->threadId);
	s += 4; /* timestamp */

	/* RsForumMsg stuff */
	s += GetTlvStringSize(item->srcId);
	s += GetTlvWideStringSize(item->title);
	s += GetTlvWideStringSize(item->msg);

	return s;
}

/* serialise the data to the buffer */
bool     RsForumSerialiser::serialiseMsg(RsForumMsg *item, void *data, uint32_t *pktsize)
{
	uint32_t tlvsize = sizeMsg(item);
	uint32_t offset = 0;

	if (*pktsize < tlvsize)
		return false; /* not enough space */

	*pktsize = tlvsize;

	bool ok = true;

	ok &= setRsItemHeader(data, tlvsize, item->PacketId(), tlvsize);

	std::cerr << "RsForumSerialiser::serialiseMsg() Header: " << ok << std::endl;
	std::cerr << "RsForumSerialiser::serialiseMsg() Size: " << tlvsize << std::endl;

	/* skip the header */
	offset += 8;

	/* RsDistribMsg first */
	ok &= SetTlvString(data, tlvsize, &offset, TLV_TYPE_STR_GROUPID, item->grpId);
	std::cerr << "RsForumSerialiser::serialiseMsg() grpId: " << ok << std::endl;
	ok &= SetTlvString(data, tlvsize, &offset, TLV_TYPE_STR_PARENTID, item->parentId);
	std::cerr << "RsForumSerialiser::serialiseMsg() parentId: " << ok << std::endl;
	ok &= SetTlvString(data, tlvsize, &offset, TLV_TYPE_STR_THREADID, item->threadId);
	std::cerr << "RsForumSerialiser::serialiseMsg() threadId: " << ok << std::endl;

	ok &= setRawUInt32(data, tlvsize, &offset, item->timestamp);
	std::cerr << "RsForumSerialiser::serialiseMsg() timestamp: " << ok << std::endl;

	/* RsForumMsg */
	ok &= SetTlvString(data, tlvsize, &offset, TLV_TYPE_STR_PEERID, item->srcId);
	std::cerr << "RsForumSerialiser::serialiseMsg() srcId: " << ok << std::endl;

	ok &= SetTlvWideString(data, tlvsize, &offset, TLV_TYPE_WSTR_TITLE, item->title);
	std::cerr << "RsForumSerialiser::serialiseMsg() Title: " << ok << std::endl;
	ok &= SetTlvWideString(data, tlvsize, &offset, TLV_TYPE_WSTR_MSG, item->msg);
	std::cerr << "RsForumSerialiser::serialiseMsg() Msg: " << ok << std::endl;

	if (offset != tlvsize)
	{
		ok = false;
		std::cerr << "RsForumSerialiser::serialiseMsg() Size Error! " << std::endl;
	}

	return ok;
}



RsForumMsg *RsForumSerialiser::deserialiseMsg(void *data, uint32_t *pktsize)
{
	/* get the type and size */
	uint32_t rstype = getRsItemId(data);
	uint32_t rssize = getRsItemSize(data);

	uint32_t offset = 0;


	if ((RS_PKT_VERSION_SERVICE != getRsItemVersion(rstype)) ||
		(RS_SERVICE_TYPE_FORUM != getRsItemService(rstype)) ||
		(RS_PKT_SUBTYPE_FORUM_MSG != getRsItemSubType(rstype)))
	{
		return NULL; /* wrong type */
	}

	if (*pktsize < rssize)    /* check size */
		return NULL; /* not enough data */

	/* set the packet length */
	*pktsize = rssize;

	bool ok = true;

	/* ready to load */
	RsForumMsg *item = new RsForumMsg();
	item->clear();

	/* skip the header */
	offset += 8;

	/* RsDistribMsg first */
	ok &= GetTlvString(data, rssize, &offset, TLV_TYPE_STR_GROUPID, item->grpId);
	ok &= GetTlvString(data, rssize, &offset, TLV_TYPE_STR_PARENTID, item->parentId);
	ok &= GetTlvString(data, rssize, &offset, TLV_TYPE_STR_THREADID, item->threadId);
	ok &= getRawUInt32(data, rssize, &offset, &(item->timestamp));

	/* RsForumMsg */
	ok &= GetTlvString(data, rssize, &offset, TLV_TYPE_STR_PEERID, item->srcId);
	ok &= GetTlvWideString(data, rssize, &offset, TLV_TYPE_WSTR_TITLE, item->title);
	ok &= GetTlvWideString(data, rssize, &offset, TLV_TYPE_WSTR_MSG, item->msg);

	if (offset != rssize)
	{
		/* error */
		delete item;
		return NULL;
	}

	if (!ok)
	{
		delete item;
		return NULL;
	}

	return item;
}


uint32_t    RsForumSerialiser::size(RsItem *item)
{
	return sizeMsg((RsForumMsg *) item);
}

bool     RsForumSerialiser::serialise(RsItem *item, void *data, uint32_t *pktsize)
{
	return serialiseMsg((RsForumMsg *) item, data, pktsize);
}

RsItem *RsForumSerialiser::deserialise(void *data, uint32_t *pktsize)
{
	return deserialiseMsg(data, pktsize);
}



/*************************************************************************/

