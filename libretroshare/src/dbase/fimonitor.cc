/*
 * RetroShare FileCache Module: fimonitor.cc
 *
 * Copyright 2004-2007 by Robert Fernie.
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

#include <util/rswin.h>
#include "dbase/fimonitor.h"
#include "util/rsdir.h"
#include "serialiser/rsserviceids.h"
#include "rsiface/rsiface.h"
#include "rsiface/rsnotify.h"
#include <errno.h>

#include <iostream>
#include <sstream>
#include <iomanip>

#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <openssl/sha.h>
#include <stdio.h>

//***********
//#define FIM_DEBUG 1
// ***********/

FileIndexMonitor::FileIndexMonitor(CacheStrapper *cs, NotifyBase *cb_in,std::string cachedir, std::string pid)
	:CacheSource(RS_SERVICE_TYPE_FILE_INDEX, false, cs, cachedir), fi(pid),
		pendingDirs(false), pendingForceCacheWrite(false),
		mForceCheck(false), mInCheck(false),cb(cb_in)

{
	updatePeriod = 60;
}


FileIndexMonitor::~FileIndexMonitor()
{
	/* Data cleanup - TODO */
	return;
}

int FileIndexMonitor::SearchKeywords(std::list<std::string> keywords, std::list<DirDetails> &results,uint32_t flags)
{
	results.clear();
	std::list<FileEntry *> firesults;

	fi.searchTerms(keywords, firesults);

	return filterResults(firesults,results,flags) ;
}

int FileIndexMonitor::SearchBoolExp(Expression *exp, std::list<DirDetails>& results,uint32_t flags) const
{
	results.clear();
	std::list<FileEntry *> firesults;

	fi.searchBoolExp(exp, firesults);

	return filterResults(firesults,results,flags) ;
}

int FileIndexMonitor::filterResults(std::list<FileEntry*>& firesults,std::list<DirDetails>& results,uint32_t flags) const
{
	/* translate/filter results */

	for(std::list<FileEntry*>::const_iterator rit(firesults.begin()); rit != firesults.end(); ++rit)
	{
		DirDetails pdetails ;
		RequestDirDetails((*rit)->parent,pdetails,0) ;
		DirDetails cdetails ;
		RequestDirDetails (*rit,cdetails,0);

		if ( ((cdetails.type == DIR_TYPE_FILE) && (pdetails.flags & flags & (DIR_FLAGS_BROWSABLE | DIR_FLAGS_NETWORK_WIDE)) > 0) ||
				((cdetails.type == DIR_TYPE_DIR) && (cdetails.flags & flags & (DIR_FLAGS_BROWSABLE | DIR_FLAGS_NETWORK_WIDE)) > 0) )
		{
			cdetails.id = "Local";
			results.push_back(cdetails);
		}
	}
	return !results.empty() ;
}

bool FileIndexMonitor::findLocalFile(std::string hash,uint32_t flags, std::string &fullpath, uint64_t &size) const
{
	std::list<FileEntry *> results;
	bool ok = false;

	fiMutex.lock(); { /* LOCKED DIRS */

#ifdef FIM_DEBUG
	std::cerr << "FileIndexMonitor::findLocalFile() Hash: " << hash << std::endl;
#endif
	/* search through the fileIndex */
	fi.searchHash(hash, results);

	if (results.size() > 0)
	{
		/* find the full path for the first entry */
		FileEntry *fe = results.front();
		DirEntry  *de = fe->parent; /* all files must have a valid parent! */

#ifdef FIM_DEBUG
		std::cerr << "FileIndexMonitor::findLocalFile() Found Name: " << fe->name << std::endl;
#endif
		std::string shpath =  RsDirUtil::removeRootDir(de->path);
		std::string basedir = RsDirUtil::getRootDir(de->path);
		std::string realroot = locked_findRealRoot(basedir);

		/* construct full name */
		if (realroot.length() > 0)
		{
			fullpath = realroot + "/";
			if (shpath != "")
			{
				fullpath += shpath + "/";
			}
			fullpath += fe->name;

			size = fe->size;
			ok = true;
		}
#ifdef FIM_DEBUG
		std::cerr << "FileIndexMonitor::findLocalFile() Found Path: " << fullpath << std::endl;
		std::cerr << "FileIndexMonitor::findLocalFile() Found Size: " << size << std::endl;
#endif
	}


	} fiMutex.unlock(); /* UNLOCKED DIRS */

	return ok;
}

bool    FileIndexMonitor::convertSharedFilePath(std::string path, std::string &fullpath)
{
	bool ok = false;

	fiMutex.lock(); { /* LOCKED DIRS */

#ifdef FIM_DEBUG
	std::cerr << "FileIndexMonitor::convertSharedFilePath() path: " << path << std::endl;
#endif

	std::string shpath =  RsDirUtil::removeRootDir(path);
	std::string basedir = RsDirUtil::getRootDir(path);
	std::string realroot = locked_findRealRoot(basedir);

	/* construct full name */
	if (realroot.length() > 0)
	{
		fullpath = realroot + "/";
		fullpath += shpath;
#ifdef FIM_DEBUG
		std::cerr << "FileIndexMonitor::convertSharedFilePath() Found Path: " << fullpath << std::endl;
#endif
		ok = true;
	}

	} fiMutex.unlock(); /* UNLOCKED DIRS */

	return ok;
}


bool FileIndexMonitor::loadLocalCache(const CacheData &data)  /* called with stored data */
{
	bool ok = false;

	fiMutex.lock(); { /* LOCKED DIRS */

	//fi.root->name = data.pid;

	/* More error checking needed here! */

	std::string name = data.name ;	// this trick allows to load the complete file. Not the one being shared.
	name[name.length()-1] = 'c' ;

	if ((ok = fi.loadIndex(data.path + '/' + data.name, data.hash, data.size)))
	{
#ifdef FIM_DEBUG
		std::cerr << "FileIndexMonitor::loadCache() Success!";
		std::cerr << std::endl;
#endif
		fi.root->row = 0;
		fi.root->name = data.pname ;
	}
	else
	{
#ifdef FIM_DEBUG
		std::cerr << "FileIndexMonitor::loadCache() Failed!";
		std::cerr << std::endl;
#endif
	}

	} fiMutex.unlock(); /* UNLOCKED DIRS */

	if (ok)
	{
		return updateCache(data);
	}
	return false;
}

bool FileIndexMonitor::updateCache(const CacheData &data)  /* we call this one */
{
	return refreshCache(data);
}


void 	FileIndexMonitor::setPeriod(int period)
{
	updatePeriod = period;
}

void 	FileIndexMonitor::run()
//void 	FileIndexMonitor::run(std::string& current_job)
{

//	updateCycle(current_job);
	updateCycle();

	while(1)
	{

		for(int i = 0; i < updatePeriod; i++)
		{

/********************************** WINDOWS/UNIX SPECIFIC PART ******************/
#ifndef WINDOWS_SYS
			sleep(1);
#else

                	Sleep(1000);
#endif
/********************************** WINDOWS/UNIX SPECIFIC PART ******************/

			/* check dirs if they've changed */
			if (internal_setSharedDirectories())
			{
				break;
			}
		}

//		updateCycle(current_job);
		updateCycle();
	}
}


//void 	FileIndexMonitor::updateCycle(std::string& current_job)
void 	FileIndexMonitor::updateCycle()
{
	time_t startstamp = time(NULL);

	/* iterate through all out-of-date directories */
	bool moretodo = true;
	bool fiMods = false;

	{
		RsStackMutex stack(fiMutex); /**** LOCKED DIRS ****/
		mInCheck = true;
	}

	while(moretodo)
	{
		/* sleep a bit for each loop */
/********************************** WINDOWS/UNIX SPECIFIC PART ******************/
#ifndef WINDOWS_SYS
		usleep(100000); /* 1/10 sec */
#else

                Sleep(100);
#endif
/********************************** WINDOWS/UNIX SPECIFIC PART ******************/

		/* check if directories have been updated */
		if (internal_setSharedDirectories())
		{
			/* reset start time */
			startstamp = time(NULL);
		}

		/* Handle a Single out-of-date directory */

		time_t stamp = time(NULL);

		/* lock dirs */
		fiMutex.lock();

        	DirEntry *olddir = fi.findOldDirectory(startstamp);

		if (!olddir)
		{
			/* finished */
			fiMutex.unlock();
			moretodo = false;
			continue;
		}

#ifdef FIM_DEBUG
		std::cerr << "FileIndexMonitor::updateCycle()";
		std::cerr << " Checking: " << olddir->path << std::endl;
#endif


        	FileEntry fe;
		/* entries that need to be checked properly */
		std::list<FileEntry> filesToHash;
		std::list<FileEntry>::iterator hit;

		/* determine the full root path */
		std::string dirpath = olddir->path;
		std::string rootdir = RsDirUtil::getRootDir(olddir->path);
		std::string remdir  = RsDirUtil::removeRootDir(olddir->path);

		std::string realroot = locked_findRealRoot(rootdir);

		std::string realpath = realroot;
		if (remdir != "")
		{
			realpath += "/" + remdir;
		}



#ifdef FIM_DEBUG
		std::cerr << "FileIndexMonitor::updateCycle()";
		std::cerr << " RealPath: " << realpath << std::endl;
#endif

		/* check for the dir existance */
		DIR *dir = opendir(realpath.c_str());
		if (!dir)
		{
#ifdef FIM_DEBUG
			std::cerr << "FileIndexMonitor::updateCycle()";
			std::cerr << " Missing Dir: " << realpath << std::endl;
#endif
			/* bad directory - delete */
                	if (!fi.removeOldDirectory(olddir->parent->path, olddir->name, stamp))
			{
				/* bad... drop out of updateCycle() - hopefully the initial cleanup
				 * will deal with it next time! - otherwise we're in a continual loop
				 */
				std::cerr << "FileIndexMonitor::updateCycle()";
                        	std::cerr << "ERROR Failed to Remove: " << olddir->path << std::endl;
			}

			fiMutex.unlock();
			continue;
		}

                /* update this dir - as its valid */
                fe.name = olddir->name;
                fi.updateDirEntry(olddir->parent->path, fe, stamp);

                /* update the directories and files here */
                std::map<std::string, DirEntry *>::iterator  dit;
                std::map<std::string, FileEntry *>::iterator fit;

		/* flag existing subdirs as old */
                for(dit = olddir->subdirs.begin(); dit != olddir->subdirs.end(); dit++)
                {
                        fe.name = (dit->second)->name;
                        /* set the age as out-of-date so that it gets checked */
                        fi.updateDirEntry(olddir->path, fe, 0);
                }

		/* now iterate through the directory...
		 * directories - flags as old,
		 * files checked to see if they have changed. (rehashed)
		 */

		struct dirent *dent;
		struct stat64 buf;

		while(NULL != (dent = readdir(dir)))
		{
			/* check entry type */
			std::string fname = dent -> d_name;
			std::string fullname = realpath + "/" + fname;
#ifdef FIM_DEBUG
			std::cerr << "calling stats on " << fullname <<std::endl;
#endif

	 		if (-1 != stat64(fullname.c_str(), &buf))
			{
#ifdef FIM_DEBUG
				std::cerr << "buf.st_mode: " << buf.st_mode <<std::endl;
#endif
	 			if (S_ISDIR(buf.st_mode))
				{
					if ((fname == ".") || (fname == ".."))
					{
#ifdef FIM_DEBUG
						std::cerr << "Skipping:" << fname << std::endl;
#endif
						continue; /* skipping links */
					}

#ifdef FIM_DEBUG
	 				std::cerr << "Is Directory: " << fullname << std::endl;
#endif

					/* add in directory */
                        		fe.name = fname;
                        		/* set the age as out-of-date so that it gets checked */
                        		fi.updateDirEntry(olddir->path, fe, 0);
				}
				else if (S_ISREG(buf.st_mode))
				{
					/* is file */
					bool toadd = false;
#ifdef FIM_DEBUG
					std::cerr << "Is File: " << fullname << std::endl;
#endif

                        		fe.name = fname;
					fe.size = buf.st_size;
					fe.modtime = buf.st_mtime;

					/* check if it exists already */
                			fit = olddir->files.find(fname);
					if (fit == olddir->files.end())
					{
						/* needs to be added */
#ifdef FIM_DEBUG
						std::cerr << "File Missing from List:" << fname << std::endl;
#endif
						toadd = true;
					}
					else
					{
						/* check size / modtime are the same */
						if ((fe.size != (fit->second)->size) ||
						    (fe.modtime != (fit->second)->modtime))
						{
#ifdef FIM_DEBUG
						std::cerr << "File ModTime/Size changed:" << fname << std::endl;
#endif
							toadd = true;
						}
						else
						{
							/* keep old info */
							fe.hash = (fit->second)->hash;
						}
					}
					if (toadd)
					{
						/* push onto Hash List */
#ifdef FIM_DEBUG
						std::cerr << "Adding to Update List: ";
						std::cerr << olddir->path;
						std::cerr << fname << std::endl;
#endif
						filesToHash.push_back(fe);
					}
					else
					{
						/* update with new time */
#ifdef FIM_DEBUG
						std::cerr << "File Hasn't Changed:" << fname << std::endl;
#endif
                        			fi.updateFileEntry(olddir->path, fe, stamp);
					}
	 			}
				else
				{
					/* unknown , ignore */
					continue;
				}
			}
#ifdef FIM_DEBUG
			else
				std::cout << "stat error " << errno << std::endl ;
#endif
		}


		/* now we unlock the lock, and iterate through the
		 * next files - hashing them, before adding into the system.
		 */
		/* for safety - blank out data we cannot use (TODO) */
		olddir = NULL;

		/* close directory */
		closedir(dir);

		/* unlock dirs */
		fiMutex.unlock();

		if (filesToHash.size() > 0)
		{
#ifdef FIM_DEBUG
			std::cerr << "List of Files to rehash in: " << dirpath << std::endl;
#endif
			fiMods = true;
			cb->notifyListPreChange(NOTIFY_LIST_DIRLIST_LOCAL, 0);
		}

#ifdef FIM_DEBUG
                for(hit = filesToHash.begin(); hit != filesToHash.end(); hit++)
		{
			std::cerr << "\t" << hit->name << std::endl;
		}

		if (filesToHash.size() > 0)
		{
			std::cerr << std::endl;
		}
#endif

		/* update files */
		for(hit = filesToHash.begin(); hit != filesToHash.end(); hit++)
		{
			//						 currentJob = "Hashing file " + realpath ;

			if (hashFile(realpath, (*hit)))
			{
				/* lock dirs */
				fiMutex.lock();

				/* update fileIndex with new time */
				/* update with new time */
				fi.updateFileEntry(dirpath, *hit, stamp);

				/* unlock dirs */
				fiMutex.unlock();
			}
			else
			{
				std::cerr << "Failed to Hash File!" << std::endl;
			}

			/* don't hit the disk too hard! */
			/********************************** WINDOWS/UNIX SPECIFIC PART ******************/
#ifndef WINDOWS_SYS
			usleep(10000); /* 1/100 sec */
#else

			Sleep(10);
#endif
			/********************************** WINDOWS/UNIX SPECIFIC PART ******************/

		}

		if (filesToHash.size() > 0)
			cb->notifyListChange(NOTIFY_LIST_DIRLIST_LOCAL, 0);
	}

	fiMutex.lock(); { /* LOCKED DIRS */

	/* finished update cycle - cleanup extra dirs/files that
	 * have not had their timestamps updated.
	 */

	if (fi.cleanOldEntries(startstamp))
	{
		//fiMods = true;
	}

	/* print out the new directory structure */

#ifdef FIM_DEBUG
	fi.printFileIndex(std::cerr);
#endif

	/* now if we have changed things -> restore file/hash it/and
	 * tell the CacheSource
	 */

	if (pendingForceCacheWrite)
	{
		pendingForceCacheWrite = false;
		fiMods = true;
	}

	}

	if (fiMods)
		locked_saveFileIndexes() ;

	fiMutex.unlock(); /* UNLOCKED DIRS */

	{
		RsStackMutex stack(fiMutex); /**** LOCKED DIRS ****/
		mInCheck = false;
	}
	cb->notifyHashingInfo("") ;
}


void FileIndexMonitor::locked_saveFileIndexes()
{
	/* store to the cacheDirectory */

	std::string path = getCacheDir();

	// Two files are saved: one with only browsable dirs, which will be shared by the cache system,
	// and one with the complete file collection.
	//
	std::ostringstream out;
	out << "fc-own-" << time(NULL) << ".rsfb";
	std::string tmpname_browsable = out.str();
	std::string fname_browsable = path + "/" + tmpname_browsable;

	std::ostringstream out2;
	out2 << "fc-own-" << time(NULL) << ".rsfc";
	std::string tmpname_total = out2.str();
	std::string fname_total = path + "/" + tmpname_total;

#ifdef FIM_DEBUG
	std::cerr << "FileIndexMonitor::updateCycle() FileIndex modified ... updating";
	std::cerr <<  std::endl;
	std::cerr << "FileIndexMonitor::updateCycle() saving browsable file list to: " << fname_browsable << std::endl ;
	std::cerr << "FileIndexMonitor::updateCycle() saving total file list to  to: " << fname_total << std::endl ;
#endif

	std::string calchash;
	uint64_t size;

	std::cerr << "About to save, with the following restrictions:" << std::endl ;
	std::set<std::string> forbidden_dirs ;
	for(std::map<std::string,SharedDirInfo>::const_iterator it(directoryMap.begin());it!=directoryMap.end();++it)
	{
		std::cerr << "   dir=" << it->first << " : " ;
		if((it->second.shareflags & RS_FILE_HINTS_BROWSABLE) == 0)
		{
			std::cerr << "forbidden" << std::endl;
			forbidden_dirs.insert(it->first) ;
		}
		else
			std::cerr << "autorized" << std::endl;
	}

	uint64_t sizetmp ;

	fi.saveIndex(fname_total, calchash, sizetmp,std::set<std::string>());	// save all files
	fi.saveIndex(fname_browsable, calchash, size,forbidden_dirs);		// save only browsable files

#ifdef FIM_DEBUG
	std::cerr << "FileIndexMonitor::updateCycle() saved with hash:" << calchash;
	std::cerr <<  std::endl;
#endif

	/* should clean up the previous cache.... */

	/* flag as new info */
	CacheData data;
	data.pid = fi.root->id;
	data.cid.type  = getCacheType();
	data.cid.subid = 0;
	data.path = path;
	data.name = tmpname_browsable;
	data.hash = calchash;
	data.size = size;
	data.recvd = time(NULL);

	updateCache(data);

#ifdef FIM_DEBUG
	std::cerr << "FileIndexMonitor::updateCycle() called updateCache()";
	std::cerr <<  std::endl;
#endif
}

void    FileIndexMonitor::updateShareFlags(const SharedDirInfo& dir)
{
	cb->notifyListPreChange(NOTIFY_LIST_DIRLIST_LOCAL, 0);

	bool fimods = false ;
#ifdef FIM_DEBUG
	std::cerr << "*** FileIndexMonitor: Updating flags for " << dir.filename << " to " << dir.shareflags << std::endl ;
#endif
	{
		RsStackMutex stack(fiMutex) ;	/* LOCKED DIRS */

		for(std::list<SharedDirInfo>::iterator it(pendingDirList.begin());it!=pendingDirList.end();++it)
		{
			std::cerr  << "** testing pending dir " << (*it).filename << std::endl ;
			if((*it).filename == dir.filename)
			{
				std::cerr  << "** Updating to " << (*it).shareflags << "!!" << std::endl ;
				(*it).shareflags = dir.shareflags ;
				break ;
			}
		}

		for(std::map<std::string,SharedDirInfo>::iterator it(directoryMap.begin());it!=directoryMap.end();++it)
		{
			std::cerr  << "** testing " << (*it).second.filename << std::endl ;
			if((*it).second.filename == dir.filename)
			{
				std::cerr  << "** Updating from " << it->second.shareflags << "!!" << std::endl ;
				(*it).second.shareflags = dir.shareflags ;
				fimods = true ;
				break ;
			}
		}
	}
	if(fimods)
	{
		RsStackMutex stack(fiMutex) ;	/* LOCKED DIRS */
		locked_saveFileIndexes() ;
	}
	cb->notifyListChange(NOTIFY_LIST_DIRLIST_LOCAL, 0);
}
	/* interface */
void    FileIndexMonitor::setSharedDirectories(std::list<SharedDirInfo> dirs)
{
	cb->notifyListPreChange(NOTIFY_LIST_DIRLIST_LOCAL, 0);

	std::list<SharedDirInfo> checkeddirs;

	std::list<SharedDirInfo>::iterator it;
#ifdef FIM_DEBUG
	std::cerr << "FileIndexMonitor::setSharedDirectories() :\n";
#endif

	for(it = dirs.begin(); it != dirs.end(); it++)
	{

#ifdef FIM_DEBUG
		std::cerr << "\t" << (*it).filename;
		std::cerr <<  std::endl;
#endif

		/* check if dir exists before adding in */
		std::string path = (*it).filename;
		DIR *dir = opendir(path.c_str());
		if (!dir)
		{
#ifdef FIM_DEBUG
			std::cerr << "FileIndexMonitor::setSharedDirectories()";
			std::cerr << " Ignoring NonExistant SharedDir: " << path << std::endl;
#endif
		}
		else
		{
			checkeddirs.push_back(*it);
		}
		closedir(dir);
	}

	RsStackMutex stack(fiMutex) ;/* LOCKED DIRS */

	pendingDirs = true;
	pendingDirList = checkeddirs;

	cb->notifyListChange(NOTIFY_LIST_DIRLIST_LOCAL, 0);
}

	/* interface */
void    FileIndexMonitor::getSharedDirectories(std::list<SharedDirInfo> &dirs)
{
	RsStackMutex stack(fiMutex) ; /* LOCKED DIRS */

	/* must provide pendingDirs, as other parts depend on instanteous response */
//	if (pendingDirs)
//		dirs = pendingDirList;
//	else
//	{
		/* get actual list (not pending stuff) */
		std::map<std::string, SharedDirInfo>::const_iterator it;

		for(it = directoryMap.begin(); it != directoryMap.end(); it++)
			dirs.push_back(it->second) ;
//	}
}


	/* interface */
void    FileIndexMonitor::forceDirectoryCheck()
{
	fiMutex.lock(); { /* LOCKED DIRS */

	if (!mInCheck)
		mForceCheck = true;

	} fiMutex.unlock(); /* UNLOCKED DIRS */
}


	/* interface */
bool    FileIndexMonitor::inDirectoryCheck()
{
	RsStackMutex stack(fiMutex); /**** LOCKED DIRS ****/

	return mInCheck;
}


bool    FileIndexMonitor::internal_setSharedDirectories()
{
	int i;
	fiMutex.lock(); /* LOCKED DIRS */

	if (!pendingDirs)
	{
		if (mForceCheck)
		{
			mForceCheck = false;
			fiMutex.unlock(); /* UNLOCKED DIRS */
			return true;
		}

		fiMutex.unlock(); /* UNLOCKED DIRS */
		return false;
	}

	mForceCheck = false;
	pendingDirs = false;
	pendingForceCacheWrite = true;

	/* clear old directories */
	directoryMap.clear();

	/* iterate through the directories */
	std::list<SharedDirInfo>::iterator it;
	std::map<std::string, SharedDirInfo>::const_iterator cit;
	for(it = pendingDirList.begin(); it != pendingDirList.end(); it++)
	{
		/* get the head directory */
		std::string root_dir = (*it).filename;
		std::string top_dir  = RsDirUtil::getTopDir(root_dir);

		/* if unique -> add, else add modifier  */
		bool unique = false;
		for(i = 0; !unique; i++)
		{
			std::string tst_dir = top_dir;
			if (i > 0)
			{
				std::ostringstream out;
				out << "-" << i;
				tst_dir += out.str();
			}
			if (directoryMap.end()== (cit=directoryMap.find(tst_dir)))
			{
				unique = true;
				/* add it! */
				directoryMap[tst_dir.c_str()] = *it;
#ifdef FIM_DEBUG
				std::cerr << "Added [" << tst_dir << "] => " << root_dir << std::endl;
#endif
			}
		}
	}

	/* now we've decided on the 'root' dirs set them to the
	 * fileIndex
	 */
	std::list<std::string> topdirs;
	for(cit = directoryMap.begin(); cit != directoryMap.end(); cit++)
	{
		topdirs.push_back(cit->first);
	}

	fi.setRootDirectories(topdirs, 0);

	locked_saveFileIndexes() ;
	fiMutex.unlock(); /* UNLOCKED DIRS */

	return true;
}




/* lookup directory function */
std::string FileIndexMonitor::locked_findRealRoot(std::string rootdir) const
{
	/**** MUST ALREADY BE LOCKED ****/
	std::string realroot = "";

	std::map<std::string, SharedDirInfo>::const_iterator cit;
	if (directoryMap.end()== (cit=directoryMap.find(rootdir)))
	{
		std::cerr << "FileIndexMonitor::locked_findRealRoot() Invalid RootDir: ";
		std::cerr << rootdir << std::endl;
	}
	else
	{
		realroot = cit->second.filename;
	}

	return realroot;
}



bool FileIndexMonitor::hashFile(std::string fullpath, FileEntry &fent)
{
	std::string f_hash = fullpath + "/" + fent.name;
	FILE *fd;
	int  len;
	SHA_CTX *sha_ctx = new SHA_CTX;
	unsigned char sha_buf[SHA_DIGEST_LENGTH];
	unsigned char gblBuf[512];

	cb->notifyHashingInfo(fent.name) ;

#ifdef FIM_DEBUG
	std::cerr << "File to hash = " << f_hash << std::endl;
#endif
	if (NULL == (fd = fopen64(f_hash.c_str(), "rb")))	return false;

	SHA1_Init(sha_ctx);
	while((len = fread(gblBuf,1, 512, fd)) > 0)
	{
		SHA1_Update(sha_ctx, gblBuf, len);
	}

	/* reading failed for some reason */
	if (ferror(fd))
	{
#ifdef FIM_DEBUG
		std::cerr << "read error !!" << std::endl;
#endif
		delete sha_ctx;
		fclose(fd);
		return false;
	}

	SHA1_Final(&sha_buf[0], sha_ctx);

	/* TODO: Actually we should store the hash data as binary ...
	 * but then it shouldn't be put in a string.
	 */

        std::ostringstream tmpout;
	for(int i = 0; i < SHA_DIGEST_LENGTH; i++)
	{
		tmpout << std::setw(2) << std::setfill('0') << std::hex << (unsigned int) (sha_buf[i]);
	}
	fent.hash = tmpout.str();

	delete sha_ctx;
	fclose(fd);
	return true;
}

int FileIndexMonitor::RequestDirDetails(std::string uid, std::string path, DirDetails &details) const
{
	/* lock it up */
	RsStackMutex mutex(fiMutex) ;

	return (uid == fi.root->id) ;
}

int FileIndexMonitor::RequestDirDetails(void *ref, DirDetails &details, uint32_t flags) const
{
	RsStackMutex mutex(fiMutex) ;

#ifdef FIM_DEBUG
	std::cerr << "FileIndexMonitor::RequestDirDetails() ref=" << ref << " flags: " << flags << std::endl;
#endif

	/* root case */

#ifdef FIM_DEBUG
	fi.root->checkParentPointers();
#endif

	// return details as reported by the FileIndex

	bool b = fi.RequestDirDetails(ref,details,flags) ;

	if(!b)
		return false ;

	// look for the top level and setup flags accordingly

	FileEntry *file = (FileEntry *) ref;
	DirEntry *dir = dynamic_cast<DirEntry *>(file);
	DirEntry *last_dir = NULL ;

	if(dir != NULL)
		while(dir->parent != NULL)
		{
			last_dir = dir ;
			dir = dir->parent ;
		}

	if(last_dir != NULL)
	{
#ifdef FIM_DEBUG
		std::cerr << "FileIndexMonitor::RequestDirDetails: parent->name=" << last_dir->name << std::endl ;
#endif
		std::map<std::string,SharedDirInfo>::const_iterator it = directoryMap.find(last_dir->name) ;

		if(it == directoryMap.end())
			std::cerr << "*********** ERROR *********** In " << __PRETTY_FUNCTION__ << std::endl ;
		else
		{
			details.flags |= (( (it->second.shareflags & RS_FILE_HINTS_BROWSABLE)>0)?DIR_FLAGS_BROWSABLE:0) ;
			details.flags |= (( (it->second.shareflags & RS_FILE_HINTS_NETWORK_WIDE)>0)?DIR_FLAGS_NETWORK_WIDE:0) ;
#ifdef FIM_DEBUG
			std::cerr << "flags = " << details.flags << std::endl ;
#endif
		}
	}

	return b ;
}


