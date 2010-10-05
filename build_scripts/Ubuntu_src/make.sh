#!/bin/sh

echo This script will
echo   - remove the directory retroshare-0.5/
echo   - remove existing sources packages in the current directory
echo   - build a new source package from the svn
echo   - rebuild the source package for the karmic i386 arch.
echo 

echo attempting to get svn revision number...
svn=`svn info | grep 'Revision:' | cut -d\  -f2`

echo Type ^C to abort, or enter to continue
read

# rm -rf ./retroshare-0.5
# ./clean.sh
# ./makeSourcePackage.sh

for dist in lucid; do
	for arch in amd64; do
		sudo PBUILDFOLDER=/var/cache/pbuilder pbuilder-dist "$dist" build *.dsc
		cp /var/cache/pbuilder/"$dist"_result/retroshare_0.5-1_"$arch".deb ./RetroShare_0.5.1."$svn"_"$dist"_"$arch".deb
	done
done

