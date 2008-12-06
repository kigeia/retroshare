/*
 * libretroshare/src/services: rsforums.cc
 *
 * RetroShare C++ Interface.
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

#include "services/p3forums.h"

uint32_t convertToInternalFlags(uint32_t extFlags);
uint32_t convertToExternalFlags(uint32_t intFlags);

std::ostream &operator<<(std::ostream &out, const ForumInfo &info)
{
	std::string name(info.forumName.begin(), info.forumName.end());
	std::string desc(info.forumDesc.begin(), info.forumDesc.end());

	out << "ForumInfo:";
	out << std::endl;
	out << "ForumId: " << info.forumId << std::endl;
	out << "ForumName: " << name << std::endl;
	out << "ForumDesc: " << desc << std::endl;
	out << "ForumFlags: " << info.forumFlags << std::endl;
	out << "Pop: " << info.pop << std::endl;
	out << "LastPost: " << info.lastPost << std::endl;

	return out;
}

std::ostream &operator<<(std::ostream &out, const ThreadInfoSummary &info)
{
	out << "ThreadInfoSummary:";
	out << std::endl;
	//out << "ForumId: " << forumId << std::endl;
	//out << "ThreadId: " << threadId << std::endl;

	return out;
}

std::ostream &operator<<(std::ostream &out, const ForumMsgInfo &info)
{
	out << "ForumMsgInfo:";
	out << std::endl;
	//out << "ForumId: " << forumId << std::endl;
	//out << "ThreadId: " << threadId << std::endl;

	return out;
}


RsForums *rsForums = NULL;


/* Forums will be initially stored for 1 year 
 * remember 2^16 = 64K max units in store period.
 * PUBPERIOD * 2^16 = max STORE PERIOD */
#define FORUM_STOREPERIOD (365*24*3600)    /* 365 * 24 * 3600 - secs in a year */
#define FORUM_PUBPERIOD   600 		   /* 10 minutes ... (max = 455 days) */

p3Forums::p3Forums(uint16_t type, CacheStrapper *cs, CacheTransfer *cft,
	                std::string srcdir, std::string storedir, 
			p3AuthMgr *mgr)
	:p3GroupDistrib(type, cs, cft, srcdir, storedir, 
		CONFIG_TYPE_FORUMS, FORUM_STOREPERIOD, FORUM_PUBPERIOD, 
		mgr), 
	mForumsChanged(false)
{ 
	//loadDummyData();
	return; 
}

p3Forums::~p3Forums() 
{ 
	return; 
}

/****************************************/

bool p3Forums::forumsChanged(std::list<std::string> &forumIds)
{
	return groupsChanged(forumIds);
}

bool p3Forums::getForumInfo(std::string fId, ForumInfo &fi)
{
	RsStackMutex stack(distribMtx); /***** STACK LOCKED MUTEX *****/

	/* extract details */
	GroupInfo *gi = locked_getGroupInfo(fId);

	if (!gi)
		return false;

	fi.forumId = gi->grpId;
	fi.forumName = gi->grpName;
	fi.forumDesc = gi->grpDesc;
	fi.forumFlags = gi->grpFlags;

	fi.subscribeFlags = gi->flags;

	fi.pop = gi->sources.size();
	fi.lastPost = gi->lastPost;

	return true;
}


bool p3Forums::getForumList(std::list<ForumInfo> &forumList)
{
	std::list<std::string> grpIds;
	std::list<std::string>::iterator it;

	getAllGroupList(grpIds);

	for(it = grpIds.begin(); it != grpIds.end(); it++)
	{
		ForumInfo fi;
		if (getForumInfo(*it, fi))
		{
			forumList.push_back(fi);
		}
	}
	return true;
}

bool p3Forums::getForumThreadList(std::string fId, std::list<ThreadInfoSummary> &msgs)
{
	std::list<std::string> msgIds;
	std::list<std::string>::iterator it;

	getParentMsgList(fId, "", msgIds);

	RsStackMutex stack(distribMtx); /***** STACK LOCKED MUTEX *****/
	for(it = msgIds.begin(); it != msgIds.end(); it++)
	{
		/* get details */
		RsDistribMsg *msg = locked_getGroupMsg(fId, *it);
		RsForumMsg *fmsg = dynamic_cast<RsForumMsg *>(msg);
		if (!fmsg)
			continue;

		ThreadInfoSummary tis;

		tis.forumId = msg->grpId;
		tis.msgId = msg->msgId;
		tis.parentId = ""; // always NULL (see request)
		tis.threadId = msg->msgId; // these are the thread heads!

		tis.ts = msg->timestamp;

		/* the rest must be gotten from the derived Msg */
		
		tis.title = fmsg->title;
		tis.msg  = fmsg->msg;

		msgs.push_back(tis);
	}
	return true;
}

bool p3Forums::getForumThreadMsgList(std::string fId, std::string pId, std::list<ThreadInfoSummary> &msgs)
{
	std::list<std::string> msgIds;
	std::list<std::string>::iterator it;

	getParentMsgList(fId, pId, msgIds);

	RsStackMutex stack(distribMtx); /***** STACK LOCKED MUTEX *****/
	for(it = msgIds.begin(); it != msgIds.end(); it++)
	{
		/* get details */
		RsDistribMsg *msg = locked_getGroupMsg(fId, *it);
		RsForumMsg *fmsg = dynamic_cast<RsForumMsg *>(msg);
		if (!fmsg)
			continue;

		ThreadInfoSummary tis;

		tis.forumId = msg->grpId;
		tis.msgId = msg->msgId;
		tis.parentId = msg->parentId;
		tis.threadId = msg->threadId;

		tis.ts = msg->timestamp;

		/* the rest must be gotten from the derived Msg */
		
		tis.title = fmsg->title;
		tis.msg  = fmsg->msg;

		msgs.push_back(tis);
	}
	return true;
}

bool p3Forums::getForumMessage(std::string fId, std::string mId, ForumMsgInfo &info)
{
	std::list<std::string> msgIds;
	std::list<std::string>::iterator it;

	RsStackMutex stack(distribMtx); /***** STACK LOCKED MUTEX *****/

	RsDistribMsg *msg = locked_getGroupMsg(fId, mId);
	RsForumMsg *fmsg = dynamic_cast<RsForumMsg *>(msg);
	if (!fmsg)
		return false;


	info.forumId = msg->grpId;
	info.msgId = msg->msgId;
	info.parentId = msg->parentId;
	info.threadId = msg->threadId;

	info.ts = msg->timestamp;

	/* the rest must be gotten from the derived Msg */
		
	info.title = fmsg->title;
	info.msg  = fmsg->msg;
	// should only use actual signature ....
	//info.srcId = fmsg->srcId;
	info.srcId = fmsg->personalSignature.keyId;

	return true;
}

bool p3Forums::ForumMessageSend(ForumMsgInfo &info)
{
	bool signIt = (info.msgflags == RS_DISTRIB_AUTHEN_REQ);

	createForumMsg(info.forumId, info.parentId, 
		info.title, info.msg, signIt);

	return true;
}


std::string p3Forums::createForum(std::wstring forumName, std::wstring forumDesc, uint32_t forumFlags)
{
        std::string id = createGroup(forumName, forumDesc, 
				convertToInternalFlags(forumFlags));

	return id;
}

std::string p3Forums::createForumMsg(std::string fId, std::string pId, 
				std::wstring title, std::wstring msg, bool signIt)
{

	RsForumMsg *fmsg = new RsForumMsg();
	fmsg->grpId = fId;
	fmsg->parentId = pId;

      {
	RsStackMutex stack(distribMtx); /***** STACK LOCKED MUTEX *****/

	RsDistribMsg *msg = locked_getGroupMsg(fId, pId);
	if (!msg)
	{
		fmsg->parentId = "";
		fmsg->threadId = "";
	}
	else
	{
		if (msg->parentId == "")
		{
			fmsg->threadId = fmsg->parentId;
		}
		else
		{
			fmsg->threadId = msg->threadId;
		}
	}
      }

	fmsg->title = title;
	fmsg->msg   = msg;
	if (signIt)
	{
		fmsg->srcId = mAuthMgr->OwnId();
	}
	fmsg->timestamp = time(NULL);

	std::string msgId = publishMsg(fmsg, signIt);
	return msgId;
}


#if 0
	/* p3Config Serialiser */
RsSerialiser *p3Forums::setupSerialiser()
{
        RsSerialiser *rss = new RsSerialiser();

	rss->addSerialType(new RsForumSerialiser());
        return rss;
}

pqistreamer *p3Forums::createStreamer(BinInterface *bio, std::string src, uint32_t bioflags)
{
        RsSerialiser *rsSerialiser = new RsSerialiser();
        //rsSerialiser->addSerialType(new RsForumSerialiser());
        rsSerialiser->addSerialType(new RsDistribSerialiser());

        pqistreamer *streamer = new pqistreamer(rsSerialiser, src, bio, bioflags);
        return streamer;
}

#endif

RsSerialType *p3Forums::createSerialiser()
{
        return new RsForumSerialiser();
}

bool    p3Forums::locked_checkDistribMsg(RsDistribMsg *msg)
{
	return true;
}


RsDistribGrp *p3Forums::locked_createPublicDistribGrp(GroupInfo &info)
{
	RsDistribGrp *grp = NULL; //new RsForumGrp();

	return grp;
}

RsDistribGrp *p3Forums::locked_createPrivateDistribGrp(GroupInfo &info)
{
	RsDistribGrp *grp = NULL; //new RsForumGrp();

	return grp;
}


uint32_t convertToInternalFlags(uint32_t extFlags)
{
	return extFlags;
}

uint32_t convertToExternalFlags(uint32_t intFlags)
{
	return intFlags;
}

bool p3Forums::forumSubscribe(std::string fId, bool subscribe)
{
	return subscribeToGroup(fId, subscribe);
}


/***************************************************************************************/
/****************** Event Feedback (Overloaded form p3distrib) *************************/
/***************************************************************************************/

#include "pqi/pqinotify.h"

void p3Forums::locked_notifyGroupChanged(GroupInfo  &grp, uint32_t flags)
{
	std::string grpId = grp.grpId;
	std::string msgId;
	std::string nullId;

        switch(flags)
        {
                case GRP_NEW_UPDATE:
                        getPqiNotify()->AddFeedItem(RS_FEED_ITEM_FORUM_NEW, grpId, msgId, nullId);
                        break;
                case GRP_UPDATE:
                        getPqiNotify()->AddFeedItem(RS_FEED_ITEM_FORUM_UPDATE, grpId, msgId, nullId);
                        break;
                case GRP_LOAD_KEY:
                        break;
                case GRP_NEW_MSG:
                        break;
                case GRP_SUBSCRIBED:
                        break;
        }
	return p3GroupDistrib::locked_notifyGroupChanged(grp, flags);
}

bool p3Forums::locked_eventDuplicateMsg(GroupInfo *grp, RsDistribMsg *msg, std::string id)
{
	return true;
}

bool p3Forums::locked_eventNewMsg(GroupInfo *grp, RsDistribMsg *msg, std::string id)
{
	std::string grpId = msg->grpId;
	std::string msgId = msg->msgId;
	std::string nullId;

	getPqiNotify()->AddFeedItem(RS_FEED_ITEM_FORUM_MSG, grpId, msgId, nullId);
	return true;
}



/****************************************/

void    p3Forums::loadDummyData()
{
	ForumInfo fi;
	std::string forumId;
	std::string msgId;
	time_t now = time(NULL);

	fi.forumId = "FID1234";
	fi.forumName = L"Forum 1";
	fi.forumDesc = L"Forum 1";
	fi.forumFlags = RS_DISTRIB_ADMIN;
	fi.pop = 2;
	fi.lastPost = now - 123;

	forumId = createForum(fi.forumName, fi.forumDesc, fi.forumFlags);

	fi.forumId = "FID2345";
	fi.forumName = L"Forum 2";
	fi.forumDesc = L"Forum 2";
	fi.forumFlags = RS_DISTRIB_SUBSCRIBED;
	fi.pop = 3;
	fi.lastPost = now - 1234;

	forumId = createForum(fi.forumName, fi.forumDesc, fi.forumFlags);
	msgId = createForumMsg(forumId, "", L"WELCOME TO Forum1", L"Hello!", true);
	msgId = createForumMsg(forumId, msgId, L"Love this forum", L"Hello2!", true);

	return; 

	/* ignore this */

	fi.forumId = "FID3456";
	fi.forumName = L"Forum 3";
	fi.forumDesc = L"Forum 3";
	fi.forumFlags = 0;
	fi.pop = 3;
	fi.lastPost = now - 1234;

	forumId = createForum(fi.forumName, fi.forumDesc, fi.forumFlags);

	fi.forumId = "FID4567";
	fi.forumName = L"Forum 4";
	fi.forumDesc = L"Forum 4";
	fi.forumFlags = 0;
	fi.pop = 5;
	fi.lastPost = now - 1234;

	forumId = createForum(fi.forumName, fi.forumDesc, fi.forumFlags);

	fi.forumId = "FID5678";
	fi.forumName = L"Forum 5";
	fi.forumDesc = L"Forum 5";
	fi.forumFlags = 0;
	fi.pop = 1;
	fi.lastPost = now - 1234;

	forumId = createForum(fi.forumName, fi.forumDesc, fi.forumFlags);

	fi.forumId = "FID6789";
	fi.forumName = L"Forum 6";
	fi.forumDesc = L"Forum 6";
	fi.forumFlags = 0;
	fi.pop = 2;
	fi.lastPost = now - 1234;

	forumId = createForum(fi.forumName, fi.forumDesc, fi.forumFlags);

	fi.forumId = "FID7890";
	fi.forumName = L"Forum 7";
	fi.forumDesc = L"Forum 7";
	fi.forumFlags = 0;
	fi.pop = 4;
	fi.lastPost = now - 1234;

	forumId = createForum(fi.forumName, fi.forumDesc, fi.forumFlags);

	fi.forumId = "FID8901";
	fi.forumName = L"Forum 8";
	fi.forumDesc = L"Forum 8";
	fi.forumFlags = 0;
	fi.pop = 3;
	fi.lastPost = now - 1234;

	forumId = createForum(fi.forumName, fi.forumDesc, fi.forumFlags);

	fi.forumId = "FID9012";
	fi.forumName = L"Forum 9";
	fi.forumDesc = L"Forum 9";
	fi.forumFlags = 0;
	fi.pop = 2;
	fi.lastPost = now - 1234;

	forumId = createForum(fi.forumName, fi.forumDesc, fi.forumFlags);

	fi.forumId = "FID9123";
	fi.forumName = L"Forum 10";
	fi.forumDesc = L"Forum 10";
	fi.forumFlags = 0;
	fi.pop = 1;
	fi.lastPost = now - 1234;

	forumId = createForum(fi.forumName, fi.forumDesc, fi.forumFlags);

	mForumsChanged = true;
}


