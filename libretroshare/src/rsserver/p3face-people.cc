
/*
 * "$Id: p3face-people.cc,v 1.8 2007-04-15 18:45:23 rmf24 Exp $"
 *
 * RetroShare C++ Interface.
 *
 * Copyright 2004-2006 by Robert Fernie.
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


#include "rsserver/p3face.h"
#include "rsserver/pqistrings.h"

#include "dht/dhthandler.h"

const int p3facepersonzone = 11451;

/*****
 *
 *
 * Friend Interface
 *
//	Basic Connect/Denied Fns 
int RsServer::FriendStatus(std::string id, bool accept);
int RsServer::FriendRemove(std::string id);
int RsServer::FriendConnectAttempt(std::string id);

//	Configuration
int RsServer::FriendSetAddress(std::string id, std::string &addr, 
		unsigned short port);
int RsServer::FriendSaveCertificate(std::string id, std::string fname); (TODO)
int RsServer::FriendSetBandwidth(std::string id, float outkB, float inkB);

//	Trust
int RsServer::FriendSignCert(std::string id);
int RsServer::FriendTrustSignature(std::string id, bool trust);

 *
 * Neighbour Interface
 *

// Not much here.
std::string RsServer::NeighGetInvite();
int     RsServer::NeighLoadPEMString(std::string pem, std::string &id);
int 	RsServer::NeighLoadCertificate(std::string fname, std::string &id);
int     RsServer::NeighAuthFriend(std::string id, RsAuthId code);
int 	RsServer::NeighAddFriend(std::string id);
int RsServer::NeighGetSigners(std::string uid, char *out, int len);

 * This file implements the Above fns from the set of RsCore
 * functions. Above are the public entry points.
 *
 * Below are the private util fns....
 * These are unique the the functions here, 
 * any common util fns are defined in p3face.cc
 *
// Update Functions.
int	RsServer::UpdateAllCerts():
int	RsServer::UpdateAllNetwork();

 * These must be run from outside the lock 
 *
void intNotifyCertError(RsCertId &id, std::string str)
void intNotifyChangeCert(RsCertId &id)
 *
 *
 */


#include "rsserver/p3face.h"

#include <iostream>
#include <sstream>

#include "pqi/pqidebug.h"
const int fltksrvrzone = 25915;

#include <sys/time.h>
#include <time.h>


/* Internal->Ext Translation */
void initRsNI(cert *c, NeighbourInfo &ni);

/* so first the cert */

/* Public Functions */

/****************************************/
	/* Public Friend Operations
	 * will lock control/iface if needed.
	 *
	 * Notifies must be outside the lock
	 * (so that it can read iface really)
	 *
	 * */


int RsServer::FriendStatus(std::string uid, bool accept)
{
	lockRsCore(); /* LOCK */

	RsCertId id(uid);

	cert *c = intFindCert(id);
	std::string errstr;
	int err = 0;
	if ((c == NULL) || (c == sslr -> getOwnCert()))
	{
		if (accept)
		{
			pqih -> cert_accept(c);
			pqih -> cert_auto(c,  true);
		}
		else
		{
			pqih -> cert_auto(c,  false);
			pqih -> cert_deny(c);
		}
	}
	else
	{
		/* error */
		err = 1;
	}
	unlockRsCore(); /* UNLOCK */

	if (err)
	{
		/* notify */
	}
	else
	{
		/* call the repopulate fn */
		UpdateAllCerts();

	}
	return err;
}



int RsServer::FriendRemove(std::string uid)
{
	lockRsCore(); /* LOCK */

	RsCertId id(uid);
	cert * c = intFindCert(id);
	int err = 0;
	if ((c == NULL) || (c == sslr -> getOwnCert()))
	{
		std::cerr << "RsServer::FriendRemove() Failed ... Bad Cert or Own." << std::endl;
		err = 1;
	}
	else
	{
		std::cerr << "RsServer::FriendRemove() " << c -> Name() << std::endl;
		pqih -> cert_deny(c);
		err = sslr -> removeCertificate(c);
	}

	unlockRsCore(); /* UNLOCK */

	if (err)
	{
		/* notify */
	}
	else
	{
		/* call the repopulate fn */
		UpdateAllCerts();

	}
	return err;
}



int RsServer::FriendConnectAttempt(std::string uid)
{
	int err = 0;

	lockRsCore(); /* LOCK */

	RsCertId id(uid);
	cert *c = intFindCert(id);
	c -> nc_timestamp = 0;
	// set Firewall to off -> so we 
	// will definitely have connect attempt.
	c -> Firewalled(false);
	c -> WillConnect(true);

	unlockRsCore(); /* UNLOCK */

	if (err)
	{
		/* notify */
	}
	else
	{
		/* call the repopulate fn */
		UpdateAllCerts();

	}
	return err;
}

int RsServer::FriendSignCert(std::string uid)
{
	lockRsCore(); /* LOCK */

	RsCertId id(uid);
	/* get the current cert, check that its
	 * in the allowed group.
	 */

	/* ask sslroot to sign certificate
	 */
	int ret = 1;
	cert *c = intFindCert(id);

	if ((c == NULL) || (c == sslr -> getOwnCert()))
	{
		ret = 0;
	}

	if (ret)
	{
		ret = sslr -> signCertificate(c);
	}

	unlockRsCore(); /* UNLOCK */

	/* notify */
	UpdateAllCerts();

	return ret;
}



int RsServer::FriendTrustSignature(std::string uid, bool trust)
{
	lockRsCore(); /* LOCK */

	RsCertId id(uid);
	int ret = -1;
	cert *c = intFindCert(id);

	if ((c == NULL) || (c == sslr -> getOwnCert()))
	{
		return -1;
	}

	ret = sslr -> trustCertificate(c, !(c -> Trusted()));


	unlockRsCore(); /* UNLOCK */

	/* notify */
	UpdateAllCerts();

	return ret;
}

/* XXX This should be changed to handle
 * (1) dns name 
 * (2) ip address.
 */
int RsServer::FriendSetFirewall(std::string uid, bool firewalled, bool forwarded)
{
	lockRsCore(); /* LOCK */
	RsCertId id(uid);

	int ret = 1;

	cert *c = intFindCert(id);
	if ((c == NULL) || (c == sslr -> getOwnCert()))
	{
		pqioutput(PQL_ALERT, fltksrvrzone, "NULL/Own cert!");
		ret = 0;
	}

	if (ret)
	{
		// get the server address.
		c -> Firewalled(firewalled);
		c -> Forwarded(forwarded);
		/* tell SSL */
		getSSLRoot() -> IndicateCertsChanged();
		ret = 1;
	}
	unlockRsCore(); /* UNLOCK */

	if (ret)
	{
		/* notify */
		UpdateAllCerts();
	}
	else
	{
		/* error message? */
		intNotifyCertError(id, "Failed to Change Cert Address");
	}
	return ret;
}

int RsServer::FriendSetLocalAddress(std::string uid, 
				std::string addr_str, 
				unsigned short port)
{
	struct sockaddr_in addr;

	lockRsCore(); /* LOCK */
	RsCertId id(uid);

	int ret = 1;

	cert *c = intFindCert(id);
	if ((c == NULL) || (c == sslr -> getOwnCert()))
	{
		pqioutput(PQL_ALERT, fltksrvrzone, "NULL/Own cert!");
		ret = 0;
	}

/********************************** WINDOWS/UNIX SPECIFIC PART *******************/
#ifndef WINDOWS_SYS
	if (ret && (0 != inet_aton(addr_str.c_str(), &(addr.sin_addr))))
#else
	addr.sin_addr.s_addr = inet_addr(addr_str.c_str());
	if (ret)
#endif
/********************************** WINDOWS/UNIX SPECIFIC PART *******************/
	{
		// get the server address.
		c -> localaddr.sin_addr = addr.sin_addr;
		c -> localaddr.sin_port = htons(port);
		/* tell SSL */
		getSSLRoot() -> IndicateCertsChanged();
		ret = 1;
	}
	unlockRsCore(); /* UNLOCK */

	if (ret)
	{
		/* notify */
		UpdateAllCerts();
	}
	else
	{
		/* error message? */
		intNotifyCertError(id, "Failed to Change Cert Address");
	}
	return ret;
}
 
int RsServer::FriendSetExtAddress(std::string uid, 
				std::string addr_str, 
				unsigned short port)
{
	struct sockaddr_in addr;

	lockRsCore(); /* LOCK */
	RsCertId id(uid);

	int ret = 1;

	cert *c = intFindCert(id);
	if ((c == NULL) || (c == sslr -> getOwnCert()))
	{
		pqioutput(PQL_ALERT, fltksrvrzone, "NULL/Own cert!");
		ret = 0;
	}

/********************************** WINDOWS/UNIX SPECIFIC PART *******************/
#ifndef WINDOWS_SYS
	if (ret && (0 != inet_aton(addr_str.c_str(), &(addr.sin_addr))))
#else
	addr.sin_addr.s_addr = inet_addr(addr_str.c_str());
	if (ret)
#endif
/********************************** WINDOWS/UNIX SPECIFIC PART *******************/
	{
		// get the server address.
		c -> serveraddr.sin_addr = addr.sin_addr;
		c -> serveraddr.sin_port = htons(port);
		/* tell SSL */
		getSSLRoot() -> IndicateCertsChanged();
		ret = 1;
	}
	unlockRsCore(); /* UNLOCK */

	if (ret)
	{
		/* notify */
		UpdateAllCerts();
	}
	else
	{
		/* error message? */
		intNotifyCertError(id, "Failed to Change Cert Address");
	}
	return ret;
}

 
int RsServer::FriendSetDNSAddress(std::string uid, std::string addr_str)
{
	lockRsCore(); /* LOCK */
	RsCertId id(uid);

	int ret = 1;

	cert *c = intFindCert(id);
	if ((c == NULL) || (c == sslr -> getOwnCert()))
	{
		pqioutput(PQL_ALERT, fltksrvrzone, "NULL/Own cert!");
		ret = 0;
	}

	if (ret)
	{
		// get the server address.
		c -> dynDNSaddr = addr_str;
		/* tell SSL */
		getSSLRoot() -> IndicateCertsChanged();
		ret = 1;
	}
	unlockRsCore(); /* UNLOCK */

	if (ret)
	{
		/* notify */
		UpdateAllCerts();
	}
	else
	{
		/* error message? */
		intNotifyCertError(id, "Failed to Change Cert Address");
	}
	return ret;
}



int RsServer::FriendSaveCertificate(std::string uid, std::string fname)
{
	lockRsCore(); /* LOCK */
	RsCertId id(uid);

	int ret = 1;

	cert *c = intFindCert(id);
	if (c == NULL) 
	{
		pqioutput(PQL_ALERT, p3facepersonzone, "FriendSaveCertificate: Invalid cert!");
		ret = 0;
	}

	if (ret)
	{
		ensureExtension(fname, "pqi");
	
		if (c -> certificate == NULL)
		{
			std::ostringstream out;
			out << "File Export (" << fname;
			out << ") Failed -> Certificate == NULL!" << std::endl;
			pqioutput(PQL_DEBUG_BASIC, p3facepersonzone, out.str());
			ret = 0;
		}
		else
		{
			ret =  sslr -> savecertificate(c, fname.c_str());
		}
	}

	unlockRsCore(); /* UNLOCK */

	return ret;
}

	
int RsServer::FriendSetBandwidth(std::string uid, float outkB, float inkB)
{
	lockRsCore(); /* LOCK */
	RsCertId id(uid);

	int ret = 1;

	/* don't do nothing yet */

	unlockRsCore(); /* UNLOCK */

	/* notify */
	UpdateAllCerts();

	return ret;
}

/********** INTERNAL PUBLIC ... 
 * a brutal but honest update.
 *
 */

int	RsServer::UpdateAllCerts()
{
	/* PreNotify of the Change */
	NotifyBase &cb = getNotify();
        cb.notifyListPreChange(NOTIFY_LIST_FRIENDS, NOTIFY_TYPE_MOD);


	lockRsCore(); /* LOCK */


	RsIface &iface = getIface();
	iface.lockData(); /* LOCK IFACE */

	int i;
	int ret = 1;

	//Fl_Funky_Browser *cb = ui -> cert_list;
	// Get the Current Cert, by getting current item 
	// and extracting ref.

	// only do if 
	if ((sslr -> CertsChanged()) ||
		(sslr -> CertsMajorChanged())) 
	{
		// clear the data.
		std::map<RsChanId, NeighbourInfo> &frnds = iface.mFriendMap;

		frnds.clear();
	
		std::list<cert *>::iterator it;
		std::list<cert *> &certs = sslr -> getCertList();
	
		std::string emptystr("");

		for(it = certs.begin(), i = 0; 
				it != certs.end(); it++, i++)
		{
			cert *c = (*it);
			NeighbourInfo ni;

			initRsNI(c, ni); /* same as for neighbour */

			frnds[ni.id] = ni;
		}

		/* let the GUI know */
		iface.setChanged(RsIface::Friend);
	}

	iface.unlockData(); /* UNLOCK IFACE */
	unlockRsCore(); /* UNLOCK */

	/* Now Notify of the Change */
        cb.notifyListChange(NOTIFY_LIST_FRIENDS, NOTIFY_TYPE_MOD);

	return ret;
}


/********* UNLOCKED INTERNAL CERT FNS....
 * These must be called from 
 * outside the lock
 */


void RsServer::intNotifyCertError(RsCertId &id, std::string str)
{
	NotifyBase &cb = getNotify();
	cb.notifyErrorMsg(NOTIFY_LIST_FRIENDS, 0, str);
	return;
}

void RsServer::intNotifyChangeCert(RsCertId &id)
{
	RsIface &iface = getIface();
	/* flag certId as modified */
	bool mods;
	iface.lockData();

	std::map<RsCertId,NeighbourInfo> &nl = iface.mNeighbourMap;
	std::map<RsCertId,NeighbourInfo>::iterator it;
	if (nl.end() != (it = nl.find(id)))
	{
		it -> second.flags = INFO_CHG;
		mods = true;

		/* Flag as changed */
		iface.setChanged(RsIface::Neighbour);
	}

	nl = iface.mFriendMap;
	//std::map<RsCertId,NeighbourInfo>::iterator it;
	if (nl.end() != (it = nl.find(id)))
	{
		it -> second.flags = INFO_CHG;
		mods = true;

		/* Flag as changed */
		iface.setChanged(RsIface::Friend);
	}


	iface.unlockData();

	/* send the notify message */
	if (mods)
	{
	  NotifyBase &cb = getNotify();
	  cb.notifyListChange(NOTIFY_LIST_NEIGHBOURS, NOTIFY_TYPE_MOD);
	  cb.notifyListChange(NOTIFY_LIST_FRIENDS, NOTIFY_TYPE_MOD);
	}
}

/********* LOCKED INTERNAL CERT FNS....
 * These must be called from within the Mutex.
 */





/**************************** NEIGHBOUR FNS *******************
 *
 *
 *
 */

/****************************************/
	/* Neighbour Operations */

std::string RsServer::NeighGetInvite()
{
	lockRsCore(); /* LOCK */

	cert *own = sslr -> getOwnCert();
	std::string name = own -> Name();
	std::string certstr = sslr -> saveCertAsString(own);

	unlockRsCore(); /* UNLOCK */

	std::ostringstream out;
	out << "You have been invited to join the retroshare";
	out << " community by: " << name;
	out << std::endl;
	out << std::endl;
	out << "Retroshare is a Friend-2-Friend network that enables you to";
	out << " communicate securely and";
	out << std::endl;
	out << "privately with your friends. ";
	out << "You can use it for chat, messages, ";
	out << "channels, and file-sharing. ";
	out << std::endl;
	out << std::endl;
	out << "Download it from: ";
	out << "http://retroshare.sf.net";
	out << std::endl;
	out << std::endl;
	out << "Thanks, ";
	out << "  " << name << ", DrBob and the RetroShare Team!";
	out << std::endl;
	out << std::endl;
	out << certstr;
	out << std::endl;

	return out.str();
}

/* same as the one below (almost) */
int     RsServer::NeighLoadPEMString(std::string pem, std::string &id)
{
	lockRsCore(); /* LOCK */

	int ret = 1;

	std::ostringstream out;
        cert *nc;

	nc = sslr -> loadCertFromString(pem);
	if (nc == NULL)
	{
		out << "Stringe Import (" << pem;
		out << ") Failed!" << std::endl;
		pqioutput(PQL_DEBUG_BASIC, fltksrvrzone, out.str());
		ret = 0;
	}
	else  if (0 > sslr -> addCollectedCertificate(nc))
	{
		out << "Imported Certificate....but no";
		out << " need for addition - As it exists!";
		out << std::endl;
		pqioutput(PQL_DEBUG_BASIC, fltksrvrzone, out.str());
		//ret = 0; want this case to set id (as is valid!)
	}

	/* ensure it gets to p3disc */
	if (ad)
	{
		ad -> distillData();
	}

	unlockRsCore(); /* UNLOCK */

	if (ret)
	{
		UpdateAllNetwork();

	 	/* set the id.  */
		RsCertId rid = intGetCertId(nc);
		std::ostringstream out;
		out << rid;
		id = out.str();
	}
	return ret;
}


int  RsServer::NeighLoadCertificate(std::string fname, std::string &id)
{
	lockRsCore(); /* LOCK */

	int ret = 1;

	std::ostringstream out;
	std::string nullhash(""); // empty hash - as signature not correct.

        cert *nc;

	nc = sslr -> loadcertificate(fname.c_str(), nullhash);
	if (nc == NULL)
	{
		out << "File Import (" << fname;
		out << ") Failed!" << std::endl;
		pqioutput(PQL_DEBUG_BASIC, fltksrvrzone, out.str());
		ret = 0;
	}
	else  if (0 > sslr -> addCollectedCertificate(nc))
	{
		out << "Imported Certificate....but no";
		out << " need for addition - As it exists!";
		out << std::endl;
		pqioutput(PQL_DEBUG_BASIC, fltksrvrzone, out.str());
		//ret = 0; want this case to set id (as is valid!)
	}

	/* ensure it gets to p3disc */
	if (ad)
	{
		ad -> distillData();
	}

	unlockRsCore(); /* UNLOCK */

	if (ret)
	{
		UpdateAllNetwork();

	 	/* set the id.  */
		RsCertId rid = intGetCertId(nc);
		std::ostringstream out;
		out << rid;
		id = out.str();
	}
	return ret;
}


int RsServer::NeighAuthFriend(std::string uid, RsAuthId code)
{
	lockRsCore(); /* LOCK */
	RsCertId id(uid);

	int ret = 1;

	cert *c = intFindCert(id);
	if ((c == NULL) || (c->certificate == NULL))
	{
		ret = 0;

	}
	else
	{
		std::string real_code = getXPGPAuthCode(c->certificate);
	
		if (code == real_code) { 
			ret = 1; 
		} else { 
			ret = 0; 
		}
	}

	/* if correct -> sign */
	if (ret)
	{
	  	if (0 < validateCertificateIsSignedByKey(c->certificate, 
			sslr -> getOwnCert() -> certificate))
		{
			std::cerr << "RsServer::cert_add_neighbour() Signed Already";
			std::cerr << std::endl;
			ret = 0;
		}
		else
		{
			/* if not signed already */
			std::cerr << "RsServer::cert_add_neighbour() Signing Cert";
			std::cerr << std::endl;

			/* sign certificate */
			sslr -> signCertificate(c);
		}
	}
	unlockRsCore(); /* UNLOCK */

	if (ret)
	{
		/* Notify of changes */
		return UpdateAllNetwork();

	}
	return (ret);
}

int 	RsServer::NeighAddFriend(std::string uid)
{
	lockRsCore(); /* LOCK */

	int ret = 1;
	RsCertId id(uid);

	cert *c = intFindCert(id);
	if ((c == NULL) || (c->certificate == NULL)
		|| (c == sslr -> getOwnCert()))
	{
		std::cerr << "RsServer::NeighAddFriend() Invalid Cert";
		std::cerr << std::endl;
		ret = 0;
	}
	else
	{
	  /* check authentication */
	  sslr -> checkAuthCertificate(c);
	  if (c->trustLvl < TRUST_SIGN_AUTHEN)
	  {
		/* do nothing */
		std::cerr << "RsServer::cert_add_neighbour() Not Authed";
		std::cerr << std::endl;
		std::cerr << "RsServer::cert_add_neighbour() Do Nothing";
		std::cerr << std::endl;
		ret = 0;
	  }
	  else
	  {
		std::cerr << "RsServer::cert_add_neighbour() Adding Cert";
		std::cerr << std::endl;

		/* add */
		sslr -> addUntrustedCertificate(c);

		std::cerr << "RsServer::cert_add_neighbour() Auto Connecting";
		std::cerr << std::endl;
		/* auto connect */
		pqih -> cert_auto(c, true);

		/* XXX: Update the Neighbour/Friends Maps */
	  }
	}

	unlockRsCore(); /* UNLOCK */

	if (ret)
	{
		/* Changed Cert Lists -> Notify */
		return UpdateAllCerts();

	}
	return (ret);
}


#if 0

std::list<std::string> 	RsServer::NeighGetSigners(std::string uid)
{
	lockRsCore(); /* LOCK */
	RsCertId id(uid);
	std::list<std::string> signers;

	int ret = 1;

	cert *c = intFindCert(id);
	if ((c == NULL) || (c->certificate == NULL))
	{
		ret = 0;

	}

	/* if valid, get signers */
	if (ret)
	{
		signers = getXPGPsigners(c->certificate);

	}
	unlockRsCore(); /* UNLOCK */

	return signers;
}

#endif



int RsServer::NeighGetSigners(std::string uid, char *out, int len)
{
	lockRsCore(); /* LOCK */
	RsCertId id(uid);
	std::list<std::string> signers;
	std::list<std::string>::iterator it;

	int ret = 1;

	cert *c = intFindCert(id);
	if ((c == NULL) || (c->certificate == NULL))
	{
		ret = 0;

	}

	/* if valid, get signers */
	if (ret)
	{
		std::string str;
		signers = getXPGPsigners(c->certificate);
		for(it = signers.begin(); it != signers.end(); it++)
		{
			str += (*it);
			str += "\n";
		}
		strncpy(out, str.c_str(), len);
		std::cerr << "SIGNERS(1): " << str << std::endl;
		std::cerr << "SIGNERS(2): " << out << std::endl;
		ret = signers.size();
	}
	unlockRsCore(); /* UNLOCK */

	return ret;
}




int  RsServer::UpdateAllNetwork()
{
	/* PreNotify of the Change */
	NotifyBase &cb = getNotify();
        cb.notifyListPreChange(NOTIFY_LIST_NEIGHBOURS, NOTIFY_TYPE_MOD);

	lockRsCore(); /* LOCK */

	RsIface &iface = getIface();
	iface.lockData(); /* LOCK IFACE */
	int ret = 1;
	int i;

	// clear the data.
	std::map<RsChanId, NeighbourInfo> &neighs = iface.mNeighbourMap;

	neighs.clear();
	
	std::list<cert *>::iterator it;
	std::list<cert *> &certs = ad -> getDiscovered();

	for(it = certs.begin(), i = 0; it != certs.end(); it++, i++)
	{
		NeighbourInfo ni;
		cert *c = (*it);
		initRsNI(c, ni);

		neighs[ni.id] = ni;
	}

	/* let the GUI know */
	iface.setChanged(RsIface::Neighbour);

	iface.unlockData(); /* UNLOCK IFACE */
	unlockRsCore(); /* UNLOCK */

	/* Now Notify of the Change */
        cb.notifyListChange(NOTIFY_LIST_NEIGHBOURS, NOTIFY_TYPE_MOD);

	return ret;
}





/******************** Translation from Int -> Ext Dtype *******************
 *
 * MUST BE UNDER LOCK!
 *
 */

void RsServer::initRsNI(cert *c, NeighbourInfo &ni)
{
	ni.id = intGetCertId(c);    /* get Id! */
	ni.name = get_cert_name(c);
	ni.org = get_cert_org(c);
	ni.loc = get_cert_loc(c);
	ni.country = get_cert_country(c);
	ni.authCode = getXPGPAuthCode(c->certificate);

	if (c->trustLvl == TRUST_SIGN_UNKNOWN)
	{
		sslr -> checkAuthCertificate(c);
	}
	ni.trustLvl = c->trustLvl;
	ni.trustString = get_trust_string(c);
	ni.statusString = get_status_string(c->Status());
	ni.connectString = get_autoconnect_string(c);
	ni.peerAddress = get_server_string(c);
        ni.lastConnect = get_lastconnecttime_string(c);

	if (c -> Accepted())
	{
		ni.acceptString = "Accept";
	}
	else
	{
		ni.acceptString = "Deny";
	}

	std::list<std::string>::iterator it;
	/**** TODO ****
	std::list<std::string> signers = getXPGPsigners(c->certificate);
	for(it = signers.begin(); it != signers.end(); it++)
	{
		ni.signIds = 
		ui -> neigh_signers -> add(it->c_str());
		std::cerr << *it << std::endl;
	}
	******/

	  /* check if we have signed the certificate */
	ni.ownsign = (0 < validateCertificateIsSignedByKey(c->certificate, 
			sslr -> getOwnCert() -> certificate));

        /* ports */
	ni.localAddr = inet_ntoa(c->localaddr.sin_addr);
	ni.localPort = ntohs(c->localaddr.sin_port);

	ni.firewalled = c -> Firewalled();
	ni.forwardPort  = c -> Forwarded();
	
	if (c -> Firewalled() && !c -> Forwarded())
	{
		ni.extAddr = "0.0.0.0";
		ni.extPort = 0;
		ni.extName = "<Incoming Not Possible>";
	}
	else
	{
		ni.extAddr = inet_ntoa(c -> serveraddr.sin_addr);
		ni.extPort = ntohs(c -> serveraddr.sin_port);
		ni.extName = c->dynDNSaddr;
	}
	
	/* data rates */
	ni.maxRate = (int) pqih -> getMaxIndivRate(true);/* kb */

	ni.inChat = c -> InChat();
	ni.inMsg  = c -> InMessage();
	
	return;
}

/* Add rest to neighbourItem later */

#if 0

	int status = c -> Status();
	if (!isfriend)
	{
		status = 0;
	}

	ui -> cert_status -> value((get_status_string(status)).c_str());

	if (isfriend)
	{
	  ui -> cert_status -> value((get_status_string(status)).c_str());
	}
	else
	{
	  ui -> cert_status -> value("Neighbour");
	}

	// SET SERVER.
	if (!cert_item_ok)
	{
	  ui->cert_server->value(inet_ntoa(c->serveraddr.sin_addr));
	  ui ->cert_port->value(ntohs(c->serveraddr.sin_port));

	  /* check if we have signed the certificate */
	  if (0 < validateCertificateIsSignedByKey(c->certificate, 
			sslr -> getOwnCert() -> certificate))

	  {
	  	ui->cert_authcode->value(
			getXPGPAuthCode(c->certificate).c_str());
	  	ui ->cert_authcode->deactivate();
	  	ui ->cert_sign_button->deactivate();
	  }
	  else
	  {
	  	ui ->cert_authcode->value("");
	  	ui ->cert_authcode->activate();
	  	ui ->cert_sign_button->deactivate();
	  }
	}

	if (c -> Manual() || (!isfriend))
		ui -> cert_auto -> value(0);
	else
		ui -> cert_auto -> value(1);

	// Finally Fill in the Text Editor.
	certtext = get_cert_info(c);

	// Last Connection 
	// Server Address
	// Status
	// Hashes

	buf = ui -> cert_details -> buffer();
	buf -> text(certtext.c_str());


	if (status & PERSON_STATUS_ACCEPTED)
		ui -> cert_allow -> value(1);
	else
		ui -> cert_allow -> value(0);

	if (status & PERSON_STATUS_WILL_LISTEN)
		ui -> cert_listen -> value(1);
	else
		ui -> cert_listen -> value(0);

	if (status & PERSON_STATUS_WILL_CONNECT)
		ui -> cert_connect -> value(1);
	else
		ui -> cert_connect -> value(0);

	if (status & PERSON_STATUS_TRUSTED)
	{
		ui -> cert_trust_person -> value(1);
	}
	else
	{
		ui -> cert_trust_person -> value(0);
	}


	cert_item_ok = true;
	return 1;
}

#endif





int RsServer::ensureExtension(std::string &name, std::string def_ext)
{
	/* if it has an extension, don't change */
	int len = name.length();
	int extpos = name.find_last_of('.');
	
	std::ostringstream out;
	out << "ensureExtension() name: " << name << std::endl;
	out << "\t\t extpos: " << extpos;
	out << " len: " << len << std::endl;
	
	/* check that the '.' has between 1 and 4 char after it (an extension) */
	if ((extpos > 0) && (extpos < len - 1) && (extpos + 6 > len))
	{
		/* extension there */
		std::string curext = name.substr(extpos, len);
		out << "ensureExtension() curext: " << curext << std::endl;
		std::cerr << out.str();
		return 0;
	}
	
	if (extpos != len - 1)
	{
		name += ".";
	}
	name += def_ext;
	
	out << "ensureExtension() added ext: " << name << std::endl;
	
	std::cerr << out.str();
	return 1;
}
	
	
