#include "ftfilecreator.h"
#include <errno.h>
#include <stdio.h>
#include <util/rsdiscspace.h>
#include <util/rsdir.h>

/*******
 * #define FILE_DEBUG 1
 ******/

#define CHUNK_MAX_AGE 40
#define MAX_FTCHUNKS_PER_PEER 5

/***********************************************************
*
*	ftFileCreator methods
*
***********************************************************/

ftFileCreator::ftFileCreator(const std::string& path, uint64_t size, const std::string& hash,bool assume_availability)
	: ftFileProvider(path,size,hash), chunkMap(size,assume_availability)
{
	/* 
         * FIXME any inits to do?
         */

#ifdef FILE_DEBUG
	std::cerr << "ftFileCreator()";
	std::cerr << std::endl;
	std::cerr << "\tpath: " << path;
	std::cerr << std::endl;
	std::cerr << "\tsize: " << size;
	std::cerr << std::endl;
	std::cerr << "\thash: " << hash;
	std::cerr << std::endl;
#endif
	RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/
	_last_recv_time_t = time(NULL) ;
}

bool ftFileCreator::getFileData(const std::string& peer_id,uint64_t offset, uint32_t &chunk_size, void *data)
{
	// Only send the data if we actually have it.
	//
#ifdef FILE_DEBUG
	std::cerr << "ftFileCreator::getFileData(). Asked for offset=" << offset << ", size=" << chunk_size << std::endl ;
#endif
	bool have_it = false ;
	{
		RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/

		have_it = chunkMap.isChunkAvailable(offset, chunk_size) ;
	}
#ifdef FILE_DEBUG
	if(have_it)
		std::cerr << "ftFileCreator::getFileData(). Have it" << std::endl ;
	else
		std::cerr << "ftFileCreator::getFileData(). Don't have it" << std::endl ;
#endif

	if(have_it)
		return ftFileProvider::getFileData(peer_id,offset, chunk_size, data);
	else
		return false ;
}

time_t ftFileCreator::lastRecvTimeStamp() 
{
	RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/
	return _last_recv_time_t ;
}
void ftFileCreator::resetRecvTimeStamp() 
{
	RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/
	_last_recv_time_t = time(NULL) ;
}

void ftFileCreator::closeFile()
{
	RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/

	if(fd != NULL)
	{
#ifdef FILE_DEBUG
		std::cerr << "CLOSED FILE " << (void*)fd << " (" << file_name << ")." << std::endl ;
#endif
		fclose(fd) ;
	}

	fd = NULL ;
}

uint64_t ftFileCreator::getRecvd()
{
	RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/
	return chunkMap.getTotalReceived() ;
}

bool ftFileCreator::addFileData(uint64_t offset, uint32_t chunk_size, void *data)
{
#ifdef FILE_DEBUG
	std::cerr << "ftFileCreator::addFileData(";
	std::cerr << offset;
	std::cerr << ", " << chunk_size;
	std::cerr << ", " << data << ")";
	std::cerr << " this: " << this;
	std::cerr << std::endl;
#endif
	/* dodgey checking outside of mutex...  much check again inside FileAttrs(). */
	/* Check File is open */

	if(!RsDiscSpace::checkForDiscSpace(RS_PARTIALS_DIRECTORY))
		return false ;

	bool complete = false ;
	{
		RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/

		if (fd == NULL)
			if (!locked_initializeFileAttrs())
				return false;

		/* 
		 * check its at the correct location 
		 */
		if (offset + chunk_size > mSize)
		{
			chunk_size = mSize - offset;
			std::cerr <<"Chunk Size greater than total file size, adjusting chunk "	<< "size " << chunk_size << std::endl;

		}

		/* 
		 * go to the offset of the file 
		 */
		if (0 != fseeko64(this->fd, offset, SEEK_SET))
		{
			std::cerr << "ftFileCreator::addFileData() Bad fseek at offset " << offset << ", fd=" << (void*)(this->fd) << ", size=" << mSize << ", errno=" << errno << std::endl;
			return 0;
		}

		if (1 != fwrite(data, chunk_size, 1, this->fd))
		{
			std::cerr << "ftFileCreator::addFileData() Bad fwrite." << std::endl;
			std::cerr << "ERRNO: " << errno << std::endl;

			return 0;
		}

#ifdef FILE_DEBUG
		std::cerr << "ftFileCreator::addFileData() added Data...";
		std::cerr << std::endl;
		std::cerr << " pos: " << offset;
		std::cerr << std::endl;
#endif
		/* 
		 * Notify ftFileChunker about chunks received 
		 */
		locked_notifyReceived(offset,chunk_size);

		complete = chunkMap.isComplete();
	}
	if(complete)
	{
#ifdef FILE_DEBUG
		std::cerr << "ftFileCreator::addFileData() File is complete: closing" << std::endl ;
#endif
		closeFile();
	}
  
	/* 
	 * FIXME HANDLE COMPLETION HERE - Any better way?
	 */

	return 1;
}

void ftFileCreator::removeInactiveChunks()
{
	RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/

#ifdef FILE_DEBUG
	std::cerr << "ftFileCreator::removeInactiveChunks(): looking for old chunks." << std::endl ;
#endif
	std::vector<ftChunk::ChunkId> to_remove ;

	chunkMap.removeInactiveChunks(to_remove) ;

#ifdef FILE_DEBUG
	if(!to_remove.empty())
		std::cerr << "ftFileCreator::removeInactiveChunks(): removing slice ids: " ;
#endif
	// This double loop looks costly, but it's called on very few chunks, and not often, so it's ok.
	//
	for(uint32_t i=0;i<to_remove.size();++i)
	{
#ifdef FILE_DEBUG
		std::cerr << to_remove[i] << " " ;
#endif
		for(std::map<uint64_t,ftChunk>::iterator it(mChunks.begin());it!=mChunks.end();)
			if(it->second.id == to_remove[i])
			{
				std::map<uint64_t,ftChunk>::iterator tmp(it) ;
				++it ;
				--mChunksPerPeer[tmp->second.peer_id].cnt ;
				mChunks.erase(tmp) ;
			}
			else
				++it ;
	}
#ifdef FILE_DEBUG
	if(!to_remove.empty())
		std::cerr << std::endl ;
#endif
}

void ftFileCreator::removeFileSource(const std::string& peer_id)
{
	RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/
#ifdef FILE_DEBUG
	std::cerr << "ftFileCreator:: removign file source " << peer_id << " from chunkmap." << std::endl ;
#endif
	chunkMap.removeFileSource(peer_id) ;
}

int ftFileCreator::locked_initializeFileAttrs()
{
#ifdef FILE_DEBUG
	std::cerr << "ftFileCreator::initializeFileAttrs() Filename: " << file_name << " this: " << this << std::endl;
#endif

	/* 
	 * check if the file exists 
	 * cant use FileProviders verion because that opens readonly.
	 */

	if (fd)
		return 1;

	/* 
	 * check if the file exists 
	 */

	{
#ifdef FILE_DEBUG
		std::cerr << "ftFileCreator::initializeFileAttrs() trying (r+b) " << file_name << " this: " << this << std::endl;
#endif
	}

	/* 
	 * attempt to open file 
	 */

	fd = fopen64(file_name.c_str(), "r+b");

	if (!fd)
	{
		std::cerr << "ftFileCreator::initializeFileAttrs() Failed to open (r+b): ";
		std::cerr << file_name << ", errno = " << errno << std::endl;

		std::cerr << "ftFileCreator::initializeFileAttrs() opening w+b";
		std::cerr << std::endl;

		/* try opening for write */
		fd = fopen64(file_name.c_str(), "w+b");
		if (!fd)
		{
			std::cerr << "ftFileCreator::initializeFileAttrs()";
			std::cerr << " Failed to open (w+b): "<< file_name << ", errno = " << errno << std::endl;
			return 0;
		}
	}
#ifdef FILE_DEBUG
	std::cerr << "OPENNED FILE " << (void*)fd << " (" << file_name << "), for r/w." << std::endl ;
#endif

	return 1;
}
ftFileCreator::~ftFileCreator()
{
	std::cerr << "Deleting file creator for " << file_name << std::endl;

	// Note: The file is actually closed in the parent, that is always a ftFileProvider.
	//
	/*
	 * FIXME Any cleanups specific to filecreator?
	 */
}


int ftFileCreator::locked_notifyReceived(uint64_t offset, uint32_t chunk_size) 
{
	/* ALREADY LOCKED */
#ifdef FILE_DEBUG
	std::cerr << "ftFileCreator::locked_notifyReceived( " << offset;
	std::cerr << ", " << chunk_size << " )";
	std::cerr << " this: " << this;
	std::cerr << std::endl;
#endif

	/* find the chunk */
	std::map<uint64_t, ftChunk>::iterator it = mChunks.find(offset);

	if (it == mChunks.end())
	{
#ifdef FILE_DEBUG
		std::cerr << "ftFileCreator::locked_notifyReceived() ";
		std::cerr << " Failed to match to existing chunk - ignoring";
		std::cerr << std::endl;

		locked_printChunkMap();
#endif
		return 0; /* ignoring */
	}

	ftChunk chunk = it->second;
	mChunks.erase(it);

	if (chunk.size != chunk_size)
	{
		/* partial : shrink chunk */
		chunk.size -= chunk_size;
		chunk.offset += chunk_size;
		mChunks[chunk.offset] = chunk;
	}
	else	// notify the chunkmap that the slice is finished, and decrement the number of chunks for this peer.
	{
		chunkMap.dataReceived(chunk.id) ;
		--mChunksPerPeer[chunk.peer_id].cnt ;
	}

	_last_recv_time_t = time(NULL) ;

	/* otherwise there is another earlier block to go
	 */

	return 1;
}

FileChunksInfo::ChunkStrategy ftFileCreator::getChunkStrategy()
{
	RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/

	return chunkMap.getStrategy() ;
}
void ftFileCreator::setChunkStrategy(FileChunksInfo::ChunkStrategy s)
{
	RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/

	// Let's check, for safety.
	if(s != FileChunksInfo::CHUNK_STRATEGY_STREAMING && s != FileChunksInfo::CHUNK_STRATEGY_RANDOM)
	{
		std::cerr << "ftFileCreator::ERROR: invalid chunk strategy " << s << "!" << " setting default value " << FileChunksInfo::CHUNK_STRATEGY_STREAMING << std::endl ;
		s = FileChunksInfo::CHUNK_STRATEGY_STREAMING ;
	}

#ifdef FILE_DEBUG
	std::cerr << "ftFileCtreator: setting chunk strategy to " << s << std::endl ;
#endif
	chunkMap.setStrategy(s) ;
}

/* Returns true if more to get 
 * But can return size = 0, if we are still waiting for the data.
 */

bool ftFileCreator::getMissingChunk(const std::string& peer_id,uint32_t size_hint,uint64_t &offset, uint32_t& size,bool& source_chunk_map_needed) 
{
	RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/
#ifdef FILE_DEBUG
	std::cerr << "ffc::getMissingChunk(...,"<< size_hint << ")";
	std::cerr << " this: " << this;
	std::cerr << std::endl;
	locked_printChunkMap();
#endif
	source_chunk_map_needed = false ;

	uint32_t& chunks_for_this_peer(mChunksPerPeer[peer_id].cnt) ;

	if(chunks_for_this_peer >= MAX_FTCHUNKS_PER_PEER)
	{
#ifdef FILE_DEBUG
		std::cerr << "ffc::getMissingChunk() too many chunks for peer " << peer_id << std::endl ;
#endif
		return false ;
	}

	/* else allocate a new chunk */

	ftChunk chunk ;

	if(!chunkMap.getDataChunk(peer_id,size_hint,chunk,source_chunk_map_needed))
		return false ;

#ifdef FILE_DEBUG
	std::cerr << "ffc::getMissingChunk() Retrieved new chunk: " << chunk << std::endl ;
#endif

	mChunks[chunk.offset] = chunk ;

	offset = chunk.offset ;
	size = chunk.size ;

	++chunks_for_this_peer ;	// increase number of chunks for this peer.

	return true; /* cos more data to get */
}

void ftFileCreator::getChunkMap(FileChunksInfo& info)
{
	RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/

	chunkMap.getChunksInfo(info) ;

	// add info pending requests, handled by ftFileCreator
	//
	info.pending_slices.clear();

	for(std::map<uint64_t, ftChunk>::iterator it = mChunks.begin();it!=mChunks.end();++it)
	{
		int n = it->second.id / info.chunk_size ;

		FileChunksInfo::SliceInfo si ;
		si.start = it->second.offset - n*info.chunk_size ;
		si.size = it->second.size ;
		si.peer_id = it->second.peer_id ;

		info.pending_slices[n].push_back(si) ;
	}
}

bool ftFileCreator::locked_printChunkMap()
{
#ifdef FILE_DEBUG
	std::cerr << "ftFileCreator::locked_printChunkMap()";
	std::cerr << " this: " << this;
	std::cerr << std::endl;
#endif

	/* check start point */
	std::cerr << "\tOutstanding Chunks:";
	std::cerr << std::endl;

	std::map<uint64_t, ftChunk>::iterator it;
	
	for(it = mChunks.begin(); it != mChunks.end(); it++)
		std::cerr << "  " << it->second << std::endl ;

	std::cerr << "Active chunks per peer:" << std::endl ;
	for(std::map<std::string,ZeroInitCounter>::const_iterator it(mChunksPerPeer.begin());it!=mChunksPerPeer.end();++it)
		std::cerr << "   " << it->first << "\t: " << it->second.cnt << std::endl;

	return true; 
}

void ftFileCreator::setAvailabilityMap(const CompressedChunkMap& cmap) 
{
	RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/

	chunkMap.setAvailabilityMap(cmap) ;
#ifdef FILE_DEBUG
	std::cerr << "ftFileCreator: setting chunkmap for hash " << hash << ": " ;

	for(uint32_t i=0;i<cmap._map.size();++i)
		std::cerr << (void*)cmap._map[i] ;
	std::cerr << std::endl ;
#endif
}

void ftFileCreator::getAvailabilityMap(CompressedChunkMap& map) 
{
	RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/

	chunkMap.getAvailabilityMap(map) ;
}

void ftFileCreator::setSourceMap(const std::string& peer_id,const CompressedChunkMap& compressed_map)
{
	RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/

	// At this point, we should cancel all file chunks that are asked to the
	// peer and which this peer actually doesn't possesses. Otherwise, the transfer may get stuck. 
	// This should be done by:
	// 	- first setting the peer availability map
	// 	- then asking the chunkmap which chunks are being downloaded, but actually shouldn't
	// 	- cancelling them in the ftFileCreator, so that they can be re-asked later to another peer.
	//
	chunkMap.setPeerAvailabilityMap(peer_id,compressed_map) ;
}

bool ftFileCreator::finished() 
{
	RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/

	return chunkMap.isComplete() ;
}

bool ftFileCreator::hashReceivedData(std::string& hash)
{
	std::cerr << "file creator " << hash << " asked for hashing received data " << file_name << std::endl;
	// csoler: No mutex here please !
	//
	// This is a bit dangerous, but otherwise we might stuck the GUI for a 
	// long time. Therefore, we must pay attention not to call this function
	// at a time file_name nor hash can be modified, which is easy.
	//
	if(!finished())
	{
		std::cerr << "Transfer not finished !! This should not happen" << std::endl;
		return false ;
	}

	uint64_t tmpsize ;
	return RsDirUtil::getFileHash(file_name,hash,tmpsize) ;
}

bool ftFileCreator::crossCheckChunkMap(const CRC32Map& ref,uint32_t& bad_chunks,uint32_t& incomplete_chunks)
{
	{
		RsStackMutex stack(ftcMutex); /********** STACK LOCKED MTX ******/

		CompressedChunkMap map ;
		chunkMap.getAvailabilityMap(map) ;
		uint32_t nb_chunks = ref.size() ;
		static const uint32_t chunk_size = ChunkMap::CHUNKMAP_FIXED_CHUNK_SIZE ;

		std::cerr << "ftFileCreator::crossCheckChunkMap(): comparing chunks..." << std::endl;

		if(!locked_initializeFileAttrs() )
			return false ;

		unsigned char *buff = new unsigned char[chunk_size] ;
		incomplete_chunks = 0 ;
		bad_chunks = 0 ;
		uint32_t len = 0 ;

		for(uint32_t i=0;i<nb_chunks;++i)
		{
			printf("  Chunk %05d/%05d:",i,nb_chunks) ;
			if(map[i])
			{
				if(fseeko64(fd,(uint64_t)i * (uint64_t)chunk_size,SEEK_SET)==0 && (len = fread(buff,1,chunk_size,fd)) > 0)
				{
					uint32_t crc = RsDirUtil::rs_CRC32(buff,len) ;

					printf(" crc: %08x, ref: %08x : ",crc,ref[i]) ;

					if(crc != ref[i])
					{
						printf(" CRC ERROR!!\n") ;
						++bad_chunks ;
						++incomplete_chunks ;
						map.reset(i) ;
					}
					else
						printf(" matched\n") ;
				}
				else
				{
					printf(" cannot fseek!\n") ;
					++bad_chunks ;
					++incomplete_chunks ;
					map.reset(i) ;
				}
			}
			else
			{
				printf(" incomplete.\n") ;
				++incomplete_chunks ;
			}
		}

		delete[] buff ;

		if(bad_chunks > 0)
			chunkMap.setAvailabilityMap(map) ;
	}
	closeFile() ;
	return true ;
}


