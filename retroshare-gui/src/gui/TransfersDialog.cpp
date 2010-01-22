/****************************************************************
 *  RetroShare is distributed under the following license:
 *
 *  Copyright (C) 2006,2007 crypton
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor,
 *  Boston, MA  02110-1301, USA.
 ****************************************************************/

#ifdef WIN32
	#include <windows.h>
#endif

#include <set>

#include "rshare.h"
#include "TransfersDialog.h"
#include "RetroShareLinkAnalyzer.h"
#include "DetailsDialog.h"
#include "DLListDelegate.h"
#include "ULListDelegate.h"
#include "FileTransferInfoWidget.h"
#include "xprogressbar.h"

#include <QContextMenuEvent>
#include <QMenu>
#include <QCursor>
#include <QPoint>
#include <QMouseEvent>
#include <QPixmap>
#include <QHeaderView>
#include <QStandardItemModel>
#include <QUrl>

#include <sstream>
#include "rsiface/rsfiles.h"
#include "rsiface/rspeers.h"
#include "rsiface/rsdisc.h"
#include "rsiface/rstypes.h"
#include <algorithm>
#include "util/misc.h"

/* Images for context menu icons */
#define IMAGE_INFO                 ":/images/fileinfo.png"
#define IMAGE_CANCEL               ":/images/delete.png"
#define IMAGE_CLEARCOMPLETED       ":/images/deleteall.png"
#define IMAGE_PLAY		             ":/images/player_play.png"
#define IMAGE_COPYLINK             ":/images/copyrslink.png"
#define IMAGE_PASTELINK            ":/images/pasterslink.png"
#define IMAGE_PAUSE					       ":/images/pause.png"
#define IMAGE_RESUME				       ":/images/resume.png"
#define IMAGE_OPENFOLDER			     ":/images/folderopen.png"
#define IMAGE_OPENFILE			       ":/images/fileopen.png"
#define IMAGE_STOP			           ":/images/stop.png"
#define IMAGE_PREVIEW			         ":/images/preview.png"
#define IMAGE_PRIORITY			       ":/images/filepriority.png"
#define IMAGE_PRIORITYLOW			     ":/images/prioritylow.png"
#define IMAGE_PRIORITYNORMAL			 ":/images/prioritynormal.png"
#define IMAGE_PRIORITYHIGH			   ":/images/priorityhigh.png"
#define IMAGE_PRIORITYAUTO			   ":/images/priorityauto.png"

//static const CompressedChunkMap *getMap_tmp()
//{
//	static CompressedChunkMap *cmap = NULL ;
//
//	if(cmap == NULL)
//	{
//		cmap = new CompressedChunkMap ; // to be passed as a parameter
//		// Initialize the chunkmap with a dummy value, for testing. To be removed...
//		cmap->_nb_chunks = 700 ;
//		cmap->_map.resize(cmap->_nb_chunks/32+1,0) ;	
//		cmap->_progress = 0.34 ;
//		for(uint i=0;i<10;++i)
//		{
//			uint32_t start = rand()%cmap->_nb_chunks;
//			uint32_t j = std::min((int)cmap->_nb_chunks,(int)start+10+(rand()%5)) ;
//
//			for(uint32_t k=start;k<j;++k)
//				COMPRESSED_MAP_WRITE(cmap->_map,k,1) ;
//		}
//
//		std::cerr << "Built cmap = " << (void*)cmap << std::endl ;
//	}
//	return cmap ;
//}


Q_DECLARE_METATYPE(FileProgressInfo) 


/** Constructor */
TransfersDialog::TransfersDialog(QWidget *parent)
: RsAutoUpdatePage(1000,parent)
{
    /* Invoke the Qt Designer generated object setup routine */
    ui.setupUi(this);

    connect( ui.downloadList, SIGNAL( customContextMenuRequested( QPoint ) ), this, SLOT( downloadListCostumPopupMenu( QPoint ) ) );

    // Set Download list model
    DLListModel = new QStandardItemModel(0,ID + 1);
    DLListModel->setHeaderData(NAME, Qt::Horizontal, tr("Name", "i.e: file name"));
    DLListModel->setHeaderData(SIZE, Qt::Horizontal, tr("Size", "i.e: file size"));
    DLListModel->setHeaderData(COMPLETED, Qt::Horizontal, tr("Completed", ""));
    DLListModel->setHeaderData(DLSPEED, Qt::Horizontal, tr("Speed", "i.e: Download speed"));
    DLListModel->setHeaderData(PROGRESS, Qt::Horizontal, tr("Progress", "i.e: % downloaded"));
    DLListModel->setHeaderData(SOURCES, Qt::Horizontal, tr("Sources", "i.e: Sources"));
    DLListModel->setHeaderData(STATUS, Qt::Horizontal, tr("Status"));
    DLListModel->setHeaderData(PRIORITY, Qt::Horizontal, tr("Priority"));
    DLListModel->setHeaderData(REMAINING, Qt::Horizontal, tr("Remaining", "i.e: Estimated Time of Arrival / Time left"));
    DLListModel->setHeaderData(ID, Qt::Horizontal, tr("Core-ID"));
    ui.downloadList->setModel(DLListModel);
    ui.downloadList->hideColumn(ID);
    DLDelegate = new DLListDelegate();
    ui.downloadList->setItemDelegate(DLDelegate);

    ui.downloadList->setAutoScroll(false) ;

  	//Selection Setup
    selection = ui.downloadList->selectionModel();

    ui.downloadList->setSelectionMode(QAbstractItemView::ExtendedSelection);

    ui.downloadList->setRootIsDecorated(true);


    /* Set header resize modes and initial section sizes Downloads TreeView*/
    QHeaderView * _header = ui.downloadList->header () ;
    _header->setResizeMode (NAME, QHeaderView::Interactive);
    _header->setResizeMode (SIZE, QHeaderView::Interactive);
    _header->setResizeMode (COMPLETED, QHeaderView::Interactive);
    _header->setResizeMode (DLSPEED, QHeaderView::Interactive);
    _header->setResizeMode (PROGRESS, QHeaderView::Interactive);
    _header->setResizeMode (SOURCES, QHeaderView::Interactive);
    _header->setResizeMode (STATUS, QHeaderView::Interactive);
    _header->setResizeMode (PRIORITY, QHeaderView::Interactive);
    _header->setResizeMode (REMAINING, QHeaderView::Interactive);

    _header->resizeSection ( NAME, 170 );
    _header->resizeSection ( SIZE, 70 );
    _header->resizeSection ( COMPLETED, 75 );
    _header->resizeSection ( DLSPEED, 75 );
    _header->resizeSection ( PROGRESS, 170 );
    _header->resizeSection ( SOURCES, 90 );
    _header->resizeSection ( STATUS, 100 );
    _header->resizeSection ( PRIORITY, 100 );
    _header->resizeSection ( REMAINING, 100 );

    connect(_header, SIGNAL(sortIndicatorChanged(int, Qt::SortOrder)), this, SLOT(saveSortIndicatorDwl(int, Qt::SortOrder)));

    // set default column and sort order for download
    _sortColDwl = 0;
    _sortOrderDwl = Qt::AscendingOrder;

    // Set Upload list model
    ULListModel = new QStandardItemModel(0,7);
    ULListModel->setHeaderData(UNAME, Qt::Horizontal, tr("Name", "i.e: file name"));
    ULListModel->setHeaderData(USIZE, Qt::Horizontal, tr("Size", "i.e: file size"));
    ULListModel->setHeaderData(USERNAME, Qt::Horizontal, tr("Peer", "i.e: user name"));
    ULListModel->setHeaderData(UPROGRESS, Qt::Horizontal, tr("Progress", "i.e: % uploaded"));
    ULListModel->setHeaderData(ULSPEED, Qt::Horizontal, tr("Speed", "i.e: upload speed"));
    ULListModel->setHeaderData(USTATUS, Qt::Horizontal, tr("Status"));
    ULListModel->setHeaderData(UTRANSFERRED, Qt::Horizontal, tr("Transferred", ""));
    ui.uploadsList->setModel(ULListModel);
    ULDelegate = new ULListDelegate();
    ui.uploadsList->setItemDelegate(ULDelegate);

    ui.uploadsList->setAutoScroll(false) ;

    ui.uploadsList->setRootIsDecorated(false);


  	//Selection Setup
	  selectionup = ui.uploadsList->selectionModel();
	  ui.uploadsList->setSelectionMode(QAbstractItemView::ExtendedSelection);

    /* Set header resize modes and initial section sizes Uploads TreeView*/
    QHeaderView * upheader = ui.uploadsList->header () ;
    upheader->setResizeMode (UNAME, QHeaderView::Interactive);
    upheader->setResizeMode (USIZE, QHeaderView::Interactive);
    upheader->setResizeMode (UTRANSFERRED, QHeaderView::Interactive);
    upheader->setResizeMode (ULSPEED, QHeaderView::Interactive);
    upheader->setResizeMode (UPROGRESS, QHeaderView::Interactive);
    upheader->setResizeMode (USTATUS, QHeaderView::Interactive);
    upheader->setResizeMode (USERNAME, QHeaderView::Interactive);

    upheader->resizeSection ( UNAME, 170 );
    upheader->resizeSection ( USIZE, 70 );
    upheader->resizeSection ( UTRANSFERRED, 75 );
    upheader->resizeSection ( ULSPEED, 75 );
    upheader->resizeSection ( UPROGRESS, 170 );
    upheader->resizeSection ( USTATUS, 100 );
    upheader->resizeSection ( USERNAME, 75 );

    connect(upheader, SIGNAL(sortIndicatorChanged(int, Qt::SortOrder)), this, SLOT(saveSortIndicatorUpl(int, Qt::SortOrder)));

    // set default column and sort order for upload
    _sortColUpl = 0;
    _sortOrderUpl = Qt::AscendingOrder;
	
    FileTransferInfoWidget *ftiw = new FileTransferInfoWidget();
    ui.fileTransferInfoWidget->setWidget(ftiw);
    ui.fileTransferInfoWidget->setWidgetResizable(true);
    ui.fileTransferInfoWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui.fileTransferInfoWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    ui.fileTransferInfoWidget->viewport()->setBackgroundRole(QPalette::NoRole);
    ui.fileTransferInfoWidget->setFrameStyle(QFrame::NoFrame);
    ui.fileTransferInfoWidget->setFocusPolicy(Qt::NoFocus);

	 QObject::connect(ui.downloadList,SIGNAL(clicked(const QModelIndex&)),this,SLOT(showFileDetails())) ;

  /* Hide platform specific features */
#ifdef Q_WS_WIN

#endif


}

void TransfersDialog::keyPressEvent(QKeyEvent *e)
{
	if(e->key() == Qt::Key_Delete)
	{
		cancel() ;
		e->accept() ;
	}
	else
		RsAutoUpdatePage::keyPressEvent(e) ;
}

void TransfersDialog::downloadListCostumPopupMenu( QPoint point )
{
	QMenu contextMnu( this );
	QMouseEvent *mevent = new QMouseEvent( QEvent::MouseButtonPress, point, Qt::RightButton, Qt::RightButton, Qt::NoModifier );

	std::set<QStandardItem *> items;
	std::set<QStandardItem *>::iterator it;
	getIdOfSelectedItems(items);

	bool single = (items.size() == 1) ;

	/* check which item is selected
	 * - if it is completed - play should appear in menu
	 */
	std::cerr << "TransfersDialog::downloadListCostumPopupMenu()" << std::endl;

	bool addPlayOption = false;

	for(int i = 0; i <= DLListModel->rowCount(); i++) {
		std::cerr << "Row Status :" << getStatus(i, DLListModel).toStdString() << ":" << std::endl;
		if(selection->isRowSelected(i, QModelIndex())) {
			std::cerr << "Selected Row Status :" << getStatus(i, DLListModel).toStdString() << ":" << std::endl;
			QString qstatus = getStatus(i, DLListModel);
			std::string status = (qstatus.trimmed()).toStdString();
			if (status == "Complete")
			{
				std::cerr << "Add Play Option" << std::endl;
				addPlayOption = true;
			}
		}
	}
	QAction *playAct = NULL;
	if (addPlayOption)
	{
		playAct = new QAction(QIcon(IMAGE_PLAY), tr( "Play" ), this );
		connect( playAct , SIGNAL( triggered() ), this, SLOT( playSelectedTransfer() ) );
	}

		QAction *detailsAct = NULL;
		pauseAct = new QAction(QIcon(IMAGE_PAUSE), tr("Pause"), this);
		connect(pauseAct, SIGNAL(triggered()), this, SLOT(pauseFileTransfer()));

		resumeAct = new QAction(QIcon(IMAGE_RESUME), tr("Resume"), this);
		connect(resumeAct, SIGNAL(triggered()), this, SLOT(resumeFileTransfer()));

		cancelAct = new QAction(QIcon(IMAGE_CANCEL), tr( "Cancel" ), this );
		connect( cancelAct , SIGNAL( triggered() ), this, SLOT( cancel() ) );

	openfolderAct = new QAction(QIcon(IMAGE_OPENFOLDER), tr("Open Folder"), this);
	connect(openfolderAct, SIGNAL(triggered()), this, SLOT(openFolderTransfer()));

		openfileAct = new QAction(QIcon(IMAGE_OPENFILE), tr("Open File"), this);
		connect(openfileAct, SIGNAL(triggered()), this, SLOT(openTransfer()));

		previewfileAct = new QAction(QIcon(IMAGE_PREVIEW), tr("Preview File"), this);
		connect(previewfileAct, SIGNAL(triggered()), this, SLOT(previewTransfer()));

		detailsfileAct = new QAction(QIcon(IMAGE_INFO), tr("Details..."), this);
		connect(detailsfileAct, SIGNAL(triggered()), this, SLOT(showDetailsDialog()));

	clearcompletedAct = new QAction(QIcon(IMAGE_CLEARCOMPLETED), tr( "Clear Completed" ), this );
	connect( clearcompletedAct , SIGNAL( triggered() ), this, SLOT( clearcompleted() ) );

#ifndef RS_RELEASE_VERSION
	copylinkAct = new QAction(QIcon(IMAGE_COPYLINK), tr( "Copy retroshare Link" ), this );
	connect( copylinkAct , SIGNAL( triggered() ), this, SLOT( copyLink() ) );
#endif

	pastelinkAct = new QAction(QIcon(IMAGE_PASTELINK), tr( "Paste retroshare Link" ), this );
	connect( pastelinkAct , SIGNAL( triggered() ), this, SLOT( pasteLink() ) );

	rootisnotdecoratedAct = new QAction(QIcon(), tr( "Set Root is not Decorated" ), this );
	connect( rootisnotdecoratedAct , SIGNAL( triggered() ), this, SLOT( rootisnotdecorated() ) );

	rootisdecoratedAct = new QAction(QIcon(), tr( "Set Root is Decorated" ), this );
	connect( rootisdecoratedAct , SIGNAL( triggered() ), this, SLOT( rootdecorated() ) );

	QMenu *viewMenu = new QMenu( tr("View"), this );
	viewMenu->addAction(rootisnotdecoratedAct);
	viewMenu->addAction(rootisdecoratedAct);

	clearQueuedDwlAct = new QAction(QIcon(), tr("Clear from Queue"), this);
	connect(clearQueuedDwlAct, SIGNAL(triggered()), this, SLOT(clearQueuedDwl()));
	clearQueueAct = new QAction(QIcon(), tr("Clear Queue"), this);
	connect(clearQueueAct, SIGNAL(triggered()), this, SLOT(clearQueue()));

	priorityLowAct = new QAction(QIcon(IMAGE_PRIORITYLOW), tr("Low"), this);
	connect(priorityLowAct, SIGNAL(triggered()), this, SLOT(priorityLow()));
	priorityNormalAct = new QAction(QIcon(IMAGE_PRIORITYNORMAL), tr("Normal"), this);
	connect(priorityNormalAct, SIGNAL(triggered()), this, SLOT(priorityNormal()));
	priorityHighAct = new QAction(QIcon(IMAGE_PRIORITYHIGH), tr("High"), this);
	connect(priorityHighAct, SIGNAL(triggered()), this, SLOT(priorityHigh()));
	priorityAutoAct = new QAction(QIcon(IMAGE_PRIORITYAUTO), tr("Auto"), this);
	connect(priorityAutoAct, SIGNAL(triggered()), this, SLOT(priorityAuto()));

		QMenu *priorityMenu = new QMenu(tr("Priority (Download)"), this);
		priorityMenu->setIcon(QIcon(IMAGE_PRIORITY));
		priorityMenu->addAction(priorityLowAct);
		priorityMenu->addAction(priorityNormalAct);
		priorityMenu->addAction(priorityHighAct);
		priorityMenu->addAction(priorityAutoAct);

		chunkStreamingAct = new QAction(QIcon(IMAGE_PRIORITYAUTO), tr("Streaming"), this);
		connect(chunkStreamingAct, SIGNAL(triggered()), this, SLOT(chunkStreaming()));
		chunkRandomAct = new QAction(QIcon(IMAGE_PRIORITYAUTO), tr("Random"), this);
		connect(chunkRandomAct, SIGNAL(triggered()), this, SLOT(chunkRandom()));

		QMenu *chunkMenu = new QMenu(tr("Chunk strategy"), this);
		chunkMenu->setIcon(QIcon(IMAGE_PRIORITY));
		chunkMenu->addAction(chunkStreamingAct);
		chunkMenu->addAction(chunkRandomAct);

	contextMnu.clear();
	if (addPlayOption)
	{
		contextMnu.addAction(playAct);
	}
	contextMnu.addSeparator();

	if(!items.empty())
	{
		bool all_paused = true ;
		bool all_downld = true ;

		QModelIndexList lst = ui.downloadList->selectionModel ()->selectedIndexes ();

		for (int i = 0; i < lst.count (); i++)
		{
			if ( lst[i].column() == 0 && lst[i].model ()->data (lst[i].model ()->index (lst[i].row (), STATUS )).toString() == "Waiting")
				all_downld = false ;
			if ( lst[i].column() == 0 && lst[i].model ()->data (lst[i].model ()->index (lst[i].row (), STATUS )).toString() == "Downloading")
				all_paused = false ;
		}

		contextMnu.addMenu( priorityMenu);
		contextMnu.addMenu( chunkMenu);

		if(!all_paused)
			contextMnu.addAction( pauseAct);
		if(!all_downld)
			contextMnu.addAction( resumeAct);
		contextMnu.addAction( cancelAct);
		contextMnu.addSeparator();
	}
#ifndef RS_RELEASE_VERSION
	if(single)
	{
		contextMnu.addAction( openfileAct);
		contextMnu.addAction( previewfileAct);
		contextMnu.addAction( openfolderAct);
		contextMnu.addAction( detailsfileAct);
		contextMnu.addSeparator();
	}
#endif
	contextMnu.addAction( clearcompletedAct);
	contextMnu.addSeparator();
#ifndef RS_RELEASE_VERSION
	contextMnu.addAction( copylinkAct);
#endif
	contextMnu.addAction( pastelinkAct);
	contextMnu.addSeparator();
	contextMnu.addAction( clearQueuedDwlAct);
	contextMnu.addAction( clearQueueAct);
	contextMnu.addSeparator();
	contextMnu.addMenu( viewMenu);

	contextMnu.exec( mevent->globalPos() );
}

void TransfersDialog::playSelectedTransfer()
{
	std::cerr << "TransfersDialog::playSelectedTransfer()" << std::endl;

        /* get the shared directories */
	std::string incomingdir = rsFiles->getDownloadDirectory();

	/* create the List of Files */
	QStringList playList;
	for(int i = 0; i <= DLListModel->rowCount(); i++) {
		if(selection->isRowSelected(i, QModelIndex())) {
			QString qstatus = getStatus(i, DLListModel);
			std::string status = (qstatus.trimmed()).toStdString();
			if (status == "Complete")
			{
				QString qname = getFileName(i, DLListModel);
				QString fullpath = QString::fromStdString(incomingdir);
				fullpath += "/";
				fullpath += qname.trimmed();

				playList.push_back(fullpath);

				std::cerr << "PlayFile:" << fullpath.toStdString() << std::endl;

			}
		}
	}
	playFiles(playList);
}


void TransfersDialog::updateProgress(int value)
{
//	for(int i = 0; i <= DLListModel->rowCount(); i++) {
//		if(selection->isRowSelected(i, QModelIndex())) {
//			editItem(i, PROGRESS, QVariant((double)value));
//		}
//	}
}

TransfersDialog::~TransfersDialog()
{
	;
}


int TransfersDialog::addItem(QString symbol, QString name, QString coreID, qlonglong fileSize, const FileProgressInfo& pinfo, double dlspeed,
		QString sources,  QString status, QString priority, qlonglong completed, qlonglong remaining)
{
    int row;
    QString sl;
    //QIcon icon(symbol);
    name.insert(0, " ");
    //sl.sprintf("%d / %d", seeds, leechs);
    row = DLListModel->rowCount();
    DLListModel->insertRow(row);

    //DLListModel->setData(DLListModel->index(row, NAME), QVariant((QIcon)icon), Qt::DecorationRole);
    DLListModel->setData(DLListModel->index(row, NAME), QVariant((QString)name), Qt::DisplayRole);
    DLListModel->setData(DLListModel->index(row, SIZE), QVariant((qlonglong)fileSize));
    DLListModel->setData(DLListModel->index(row, COMPLETED), QVariant((qlonglong)completed));
    DLListModel->setData(DLListModel->index(row, DLSPEED), QVariant((double)dlspeed));
	 DLListModel->setData(DLListModel->index(row, PROGRESS), QVariant::fromValue(pinfo));
    DLListModel->setData(DLListModel->index(row, SOURCES), QVariant((QString)sources));
    DLListModel->setData(DLListModel->index(row, STATUS), QVariant((QString)status));
    DLListModel->setData(DLListModel->index(row, PRIORITY), QVariant((QString)priority));
    DLListModel->setData(DLListModel->index(row, REMAINING), QVariant((qlonglong)remaining));
    DLListModel->setData(DLListModel->index(row, ID), QVariant((QString)coreID));


    QString ext = QFileInfo(name).suffix();
    if (ext == "jpg" || ext == "jpeg" || ext == "tif" || ext == "tiff" || ext == "JPG"|| ext == "png" || ext == "gif"
    || ext == "bmp" || ext == "ico" || ext == "svg")
    {
      DLListModel->setData(DLListModel->index(row,NAME), QIcon(QString::fromUtf8(":/images/FileTypePicture.png")), Qt::DecorationRole);
    }
    else if (ext == "avi" ||ext == "AVI" || ext == "mpg" || ext == "mpeg" || ext == "wmv" || ext == "divx" || ext == "ts"
    || ext == "mkv" || ext == "mp4" || ext == "flv" || ext == "mov" || ext == "asf" || ext == "xvid"
    || ext == "vob" || ext == "qt" || ext == "rm" || ext == "3gp" || ext == "mpeg" || ext == "ogm")
    {
    DLListModel->setData(DLListModel->index(row,NAME), QIcon(QString::fromUtf8(":/images/FileTypeVideo.png")), Qt::DecorationRole);
    }
    else if (ext == "ogg" || ext == "mp3" || ext == "MP3"  || ext == "mp1" || ext == "mp2" || ext == "wav" || ext == "wma")
    {
    DLListModel->setData(DLListModel->index(row,NAME), QIcon(QString::fromUtf8(":/images/FileTypeAudio.png")), Qt::DecorationRole);
    }
    else if (ext == "tar" || ext == "bz2" || ext == "zip" || ext == "gz" || ext == "7z" || ext == "msi"
    || ext == "rar" || ext == "rpm" || ext == "ace" || ext == "jar" || ext == "tgz" || ext == "lha"
    || ext == "cab" || ext == "cbz"|| ext == "cbr" || ext == "alz" || ext == "sit" || ext == "arj" || ext == "deb")
    {
    DLListModel->setData(DLListModel->index(row,NAME), QIcon(QString::fromUtf8(":/images/FileTypeArchive.png")), Qt::DecorationRole);
    }
    else if (ext == "app" || ext == "bat" || ext == "cgi" || ext == "com"
    || ext == "exe" || ext == "js" || ext == "pif"
    || ext == "py" || ext == "pl" || ext == "sh" || ext == "vb" || ext == "ws")
    {
    DLListModel->setData(DLListModel->index(row,NAME), QIcon(QString::fromUtf8(":/images/FileTypeProgram.png")), Qt::DecorationRole);
    }
    else if (ext == "iso" || ext == "nrg" || ext == "mdf" || ext == "img" || ext == "dmg" || ext == "bin" )
    {
    DLListModel->setData(DLListModel->index(row,NAME), QIcon(QString::fromUtf8(":/images/FileTypeCDImage.png")), Qt::DecorationRole);
    }
    else if (ext == "txt" || ext == "cpp" || ext == "c" || ext == "h")
    {
    DLListModel->setData(DLListModel->index(row,NAME), QIcon(QString::fromUtf8(":/images/FileTypeDocument.png")), Qt::DecorationRole);
    }
    else if (ext == "doc" || ext == "rtf" || ext == "sxw" || ext == "xls" || ext == "pps" || ext == "xml"
    || ext == "sxc" || ext == "odt" || ext == "ods" || ext == "dot" || ext == "ppt" || ext == "css"  )
    {
    DLListModel->setData(DLListModel->index(row,NAME), QIcon(QString::fromUtf8(":/images/FileTypeDocument.png")), Qt::DecorationRole);
    }
    else if (ext == "html" || ext == "htm" || ext == "php")
    {
    DLListModel->setData(DLListModel->index(row,NAME), QIcon(QString::fromUtf8(":/images/FileTypeDocument.png")), Qt::DecorationRole);
    }
    else
    {
    DLListModel->setData(DLListModel->index(row,NAME), QIcon(QString::fromUtf8(":/images/FileTypeAny.png")), Qt::DecorationRole);
    }

	return row;
}

bool TransfersDialog::addPeerToItem(int row, QString symbol, QString name, QString coreID, qlonglong fileSize, const FileProgressInfo& pinfo, double dlspeed, QString sources, QString status, qlonglong completed, qlonglong remaining)
{
    QStandardItem *dlItem = DLListModel->item(row);
    if (!dlItem) return false;

    //set this false if you want to expand on double click
    dlItem->setEditable(false);

    name.insert(0, " ");

    QList<QStandardItem *> items;
    QStandardItem *i1 = new QStandardItem(); i1->setData(QVariant((QString)name), Qt::DisplayRole);
    QStandardItem *i2 = new QStandardItem(); i2->setData(QVariant((qlonglong)fileSize), Qt::DisplayRole);
    QStandardItem *i3 = new QStandardItem(); i3->setData(QVariant((qlonglong)completed), Qt::DisplayRole);
    QStandardItem *i4 = new QStandardItem(); i4->setData(QVariant((double)dlspeed), Qt::DisplayRole);
    QStandardItem *i5 = new QStandardItem(); i5->setData(QVariant::fromValue(pinfo), Qt::DisplayRole);
    QStandardItem *i6 = new QStandardItem(); i6->setData(QVariant((QString)sources), Qt::DisplayRole);
    QStandardItem *i7 = new QStandardItem(); i7->setData(QVariant((QString)status), Qt::DisplayRole);
    QStandardItem *i8 = new QStandardItem(); i8->setData(QVariant((QString)tr("")), Qt::DisplayRole);	// blank field for priority
    QStandardItem *i9 = new QStandardItem(); i9->setData(QVariant((qlonglong)remaining), Qt::DisplayRole);
    QStandardItem *i10 = new QStandardItem(); i10->setData(QVariant((QString)coreID), Qt::DisplayRole);

    /* set status icon in the name field */
    if (status == "Downloading") {
        i1->setData(QIcon(QString::fromUtf8(":/images/Client0.png")), Qt::DecorationRole);
    } else if (status == "Failed") {
        i1->setData(QIcon(QString::fromUtf8(":/images/Client1.png")), Qt::DecorationRole);
    } else if (status == "Okay") {
        i1->setData(QIcon(QString::fromUtf8(":/images/Client2.png")), Qt::DecorationRole);
    } else if (status == "Waiting") {
        i1->setData(QIcon(QString::fromUtf8(":/images/Client3.png")), Qt::DecorationRole);
    } else if (status == "Unknown") {
        i1->setData(QIcon(QString::fromUtf8(":/images/Client4.png")), Qt::DecorationRole);
    } else if (status == "Complete") {
    }

    items.append(i1);
    items.append(i2);
    items.append(i3);
    items.append(i4);
    items.append(i5);
    items.append(i6);
    items.append(i7);
    items.append(i8);
    items.append(i9);
    items.append(i10);

    dlItem->appendRow(items);

    return true;
}


int TransfersDialog::addUploadItem(QString symbol, QString name, QString coreID, qlonglong fileSize, const FileProgressInfo& pinfo, double dlspeed, QString source,  QString status, qlonglong completed, qlonglong remaining)
{
	int row;
	QString sl;
	//QIcon icon(symbol);
	name.insert(0, " ");
	row = ULListModel->rowCount();
	ULListModel->insertRow(row);

	ULListModel->setData(ULListModel->index(row, UNAME), QVariant((QString)name), Qt::DisplayRole);
	ULListModel->setData(ULListModel->index(row, USIZE), QVariant((qlonglong)fileSize));
	ULListModel->setData(ULListModel->index(row, UTRANSFERRED), QVariant((qlonglong)completed));
	ULListModel->setData(ULListModel->index(row, ULSPEED), QVariant((double)dlspeed));
	ULListModel->setData(ULListModel->index(row, UPROGRESS), QVariant::fromValue(pinfo));
	ULListModel->setData(ULListModel->index(row, USTATUS), QVariant((QString)status));
	ULListModel->setData(ULListModel->index(row, USERNAME), QVariant((QString)source));

	return row;
}


void TransfersDialog::delItem(int row)
{
	DLListModel->removeRow(row, QModelIndex());
}

void TransfersDialog::delUploadItem(int row)
{
	ULListModel->removeRow(row, QModelIndex());
}

void TransfersDialog::editItem(int row, int column, QVariant data)
{
	//QIcon *icon;
	switch(column) {
		//case SYMBOL:
		//	icon = new QIcon(data.toString());
		//	DLListModel->setData(DLListModel->index(row, NAME), QVariant((QIcon)*icon), Qt::DecorationRole);
		//	delete icon;
		//	break;
		case NAME:
			DLListModel->setData(DLListModel->index(row, NAME), data, Qt::DisplayRole);
			break;
		case SIZE:
			DLListModel->setData(DLListModel->index(row, SIZE), data);
			break;
//		case PROGRESS:
//			DLListModel->setData(DLListModel->index(row, PROGRESS), data);
//			break;
		case DLSPEED:
			DLListModel->setData(DLListModel->index(row, DLSPEED), data);
			break;
		case SOURCES:
			DLListModel->setData(DLListModel->index(row, SOURCES), data);
			break;
		case STATUS:
			DLListModel->setData(DLListModel->index(row, STATUS), data);
			break;
		case PRIORITY:
			DLListModel->setData(DLListModel->index(row, PRIORITY), data);
			break;
		case COMPLETED:
			DLListModel->setData(DLListModel->index(row, COMPLETED), data);
			break;
		case REMAINING:
			DLListModel->setData(DLListModel->index(row, REMAINING), data);
			break;
		case ID:
			DLListModel->setData(DLListModel->index(row, ID), data);
			break;
	}
}

	/* get the list of Transfers from the RsIface.  **/
void TransfersDialog::updateDisplay()
{
	insertTransfers();
}
void TransfersDialog::insertTransfers()
{
	QString symbol, name, sources, status, priority, coreId;
	qlonglong fileSize, completed, remaining;
	double progress, dlspeed;


	/* get current selection */
	std::list<std::string> selectedIds;

	for (int i = 0; i < DLListModel->rowCount(); i++) {
	    if (selection->isRowSelected(i, QModelIndex())) {
	        std::string id = getID(i, DLListModel).toStdString();
	        selectedIds.push_back(id);
        }

        QStandardItem *parent = DLListModel->item(i);
        //if (!parent) continue;
        int childs = parent->rowCount();
        for (int j = 0; j < childs; j++) {
            QModelIndex parentIndex = DLListModel->indexFromItem(parent);
            if (selection->isRowSelected(j, parentIndex)) {
                QStandardItem *child = parent->child(j, ID);
                std::string id = child->data(Qt::DisplayRole).toString().toStdString();
                selectedIds.push_back(id);
            }
        }
    }

	/* view will be scrolled to first selected index */
	QModelIndex firstSelIdx;

	/* get expanded donwload items */
	std::list<std::string> expandedIds;

	for (int i = 0; i < DLListModel->rowCount(); i++) {
	    QStandardItem *item = DLListModel->item(i);
	    QModelIndex index = DLListModel->indexFromItem(item);
	    if (ui.downloadList->isExpanded(index)) {
	        std::string id = getID(i, DLListModel).toStdString();
	        expandedIds.push_back(id);
        }
    }

	//remove all Items
	for(int i = DLListModel->rowCount(); i >= 0; i--)
	{
		delItem(i);
	}

	for(int i = ULListModel->rowCount(); i >= 0; i--)
	{
		delUploadItem(i);
	}

	ui.downloadList->sortByColumn(_sortColDwl, _sortOrderDwl);
	/* disable for performance issues, enable after insert all transfers */
	ui.downloadList->setSortingEnabled(false);

	/* get the download and upload lists */
	std::list<std::string> downHashes;
	std::list<std::string> upHashes;

	rsFiles->FileDownloads(downHashes);
	rsFiles->FileUploads(upHashes);

	uint32_t dlCount = 0;
	uint32_t ulCount = 0;

    std::list<std::string>::iterator it;
    for (it = downHashes.begin(); it != downHashes.end(); it++) 
	 {
        FileInfo info;
        if (!rsFiles->FileDetails(*it, RS_FILE_HINTS_DOWNLOAD, info)) continue;
        //if file transfer is a cache file index file, don't show it
        if (info.flags & CB_CODE_CACHE) continue;

        symbol      = "";
        name        = QString::fromUtf8(info.fname.c_str());
        coreId      = QString::fromStdString(info.hash);
        fileSize    = info.size;
        progress    = (info.transfered * 100.0) / info.size;
        dlspeed     = info.tfRate * 1024.0;

        /* get number of online peers */
        int online = 0;
        std::list<TransferInfo>::iterator pit;
        for (pit = info.peers.begin(); pit != info.peers.end(); pit++) {
            if (rsPeers->isOnline(pit->peerId)) {
                online++;
            }
        }

        sources     = QString("%1 (%2)").arg(online).arg(info.peers.size() - online);

        switch (info.downloadStatus) {
            case FT_STATE_FAILED:
                status = tr("Failed"); break;
            case FT_STATE_OKAY:
                status = tr("Okay"); break;
            case FT_STATE_WAITING:
                status = tr("Waiting"); break;
            case FT_STATE_DOWNLOADING:
                status = tr("Downloading"); break;
            case FT_STATE_COMPLETE:
                status = tr("Complete"); break;
            default:
                status = tr("Unknown"); break;
        }

        priority	= "";	/* for already downloading files */
        completed   = info.transfered;
        remaining   = (info.size - info.transfered) / (info.tfRate * 1024.0);

		  FileChunksInfo fcinfo ;
		  if(!rsFiles->FileDownloadChunksDetails(*it,fcinfo)) 
			  continue ;

		  FileProgressInfo pinfo ;
		  pinfo.cmap = fcinfo.chunks ;
		  pinfo.progress = completed*100.0/info.size ;

//			std::cerr << "Converting fcinfo to compressed chunk map. Chunks=" << fcinfo.chunks.size() << std::endl ;
//			std::cerr << "map data = " ;
//			for(uint k=0;k<cmap._map.size();++k)
//				std::cout << (void*)cmap._map[k] ;
//			std::cout << std::endl ;

        int addedRow = addItem(symbol, name, coreId, fileSize, pinfo, dlspeed, sources, status, priority, completed, remaining);

        /* if found in selectedIds -> select again */
        if (selectedIds.end() != std::find(selectedIds.begin(), selectedIds.end(), info.hash)) {
            selection->select(DLListModel->index(dlCount, NAME),
                QItemSelectionModel::Rows | QItemSelectionModel::SelectCurrent);
            if (!firstSelIdx.isValid())
            	firstSelIdx = DLListModel->index(dlCount, NAME);
        }

        /* expand last expanded items */
        if (expandedIds.end() != std::find(expandedIds.begin(), expandedIds.end(), info.hash)) {
            QStandardItem *item = DLListModel->item(dlCount);
            QModelIndex index = DLListModel->indexFromItem(item);
            ui.downloadList->setExpanded(index, true);
        }

        dlCount++;

        /* continue to next download item if no peers to add */
        if (!info.peers.size()) continue;

        std::map<std::string, std::string>::iterator vit;
        std::map<std::string, std::string> versions;
        bool retv = rsDisc->getDiscVersions(versions);

        int dlPeers = 0;
        for (pit = info.peers.begin(); pit != info.peers.end(); pit++) {
            symbol      = "";
            name        = getPeerName(pit->peerId);
            //unique combination: fileName + peerName, variant: hash + peerName (too long)
            coreId      = QString::fromStdString(info.fname) + getPeerName(pit->peerId);
            fileSize    = info.size;
            progress    = (info.transfered * 100.0) / info.size;
            sources     = "";
            if (retv && versions.end() != (vit = versions.find(pit->peerId))) {
            	sources		= QString("rShare v") + QString::fromStdString(vit->second);
            }

            if (pit->status == FT_STATE_COMPLETE) {
                status = tr("Complete");
            } else {
                status = tr("Unknown");
                switch (pit->status) {
                    case FT_STATE_FAILED:
                        status = tr("Failed"); break;
                    case FT_STATE_OKAY:
                        status = tr("Okay"); break;
                    case FT_STATE_WAITING:
                        status = tr("Waiting"); break;
                    case FT_STATE_DOWNLOADING:
                        status = tr("Downloading"); break;
                    case FT_STATE_COMPLETE:
                        status = tr("Complete"); break;
                }
            }

            dlspeed			= 0;
            if (status == "Downloading")
            	dlspeed     = pit->tfRate * 1024.0;

            completed   = info.transfered;
            remaining   = (info.size - info.transfered) / (pit->tfRate * 1024.0);

				FileProgressInfo pinfo ;
				pinfo.cmap = fcinfo.compressed_peer_availability_maps[pit->peerId] ;
				pinfo.progress = 0.0 ;	// we don't display completion for sources.

            if (!addPeerToItem(addedRow, symbol, name, coreId, fileSize, pinfo, dlspeed, sources, status, completed, remaining))
                continue;

            /* if peers found in selectedIds, select again */
            if (selectedIds.end() != std::find(selectedIds.begin(), selectedIds.end(), info.fname + getPeerName(pit->peerId).toStdString())) {
                QStandardItem *dlItem = DLListModel->item(addedRow);
                QModelIndex childIndex = DLListModel->indexFromItem(dlItem).child(dlPeers, 0);
                selection->select(childIndex, QItemSelectionModel::Rows | QItemSelectionModel::SelectCurrent);
            }
            dlPeers++;
        }
    }

    /* here i will insert files from the download queue - which are
     * not started yet and can't be find in FileDownloads
     * */
    std::list<DwlDetails> details;
    std::list<DwlDetails>::iterator dit;
    rsFiles->getDwlDetails(details);
    for (dit = details.begin(); dit != details.end(); dit ++)
    {
    	name 		= QString::fromUtf8(dit->fname.c_str());
    	coreId 		= QString::fromStdString(dit->hash);
    	fileSize 	= dit->count;
    	progress    = 0;
    	dlspeed     = 0;
    	sources     = "";
    	completed   = 0;
    	status 		= tr("Queued");
    	remaining   = 0;

		switch (dit->priority) {
			case 0:
				priority = tr("Low");
				break;
			case 1:
				priority = tr("Normal");
				break;
			case 2:
				priority = tr("High");
				break;
			case 3:
				priority = tr("Auto");
				break;
			default:
				priority = tr("Auto");
				break;
		}

		FileProgressInfo pinfo ;
		pinfo.progress = 0.0 ;

		addItem("", name, coreId, fileSize, pinfo, dlspeed, sources, status, priority, completed, remaining);

		/* if found in selectedIds -> select again */
		if (selectedIds.end() != std::find(selectedIds.begin(), selectedIds.end(), dit->hash)) {
			selection->select(DLListModel->index(dlCount, NAME),
				QItemSelectionModel::Rows | QItemSelectionModel::SelectCurrent);
			if (!firstSelIdx.isValid())
				firstSelIdx = DLListModel->index(dlCount, NAME);
		}

		dlCount++;
    }

    /* scroll to first selected index if any */
    if (firstSelIdx.isValid())
    	ui.downloadList->scrollTo(firstSelIdx);

    ui.downloadList->setSortingEnabled(true);

    ui.uploadsList->sortByColumn(_sortColUpl, _sortOrderUpl);
	/* disable for performance issues, enable after insert all transfers */
	ui.uploadsList->setSortingEnabled(false);
	for(it = upHashes.begin(); it != upHashes.end(); it++)
	{
	  FileInfo info;
	  if (!rsFiles->FileDetails(*it, RS_FILE_HINTS_UPLOAD, info))
	  {
		continue;
	  }

	  std::list<TransferInfo>::iterator pit;
	  for(pit = info.peers.begin(); pit != info.peers.end(); pit++)
	  {
		symbol  	= "";
		coreId		= QString::fromStdString(info.hash);
		name    	= QString::fromUtf8(info.fname.c_str());
		sources		= getPeerName(pit->peerId);

		switch(pit->status)
		{
			case FT_STATE_FAILED:
				status = tr("Failed");
				break;
			case FT_STATE_OKAY:
				status = tr("Okay");
				break;
			case FT_STATE_WAITING:
				status = tr("Waiting");
				break;
			case FT_STATE_DOWNLOADING:
				status = tr("Uploading");
				break;
		    	case FT_STATE_COMPLETE:
			default:
				status = tr("Complete");
				break;

        	}

	//	if (info.downloadStatus == FT_STATE_COMPLETE)
	//	{
	//		status = "Complete";
	//	}

		FileProgressInfo pinfo ;

		if(!rsFiles->FileUploadChunksDetails(*it,pit->peerId,pinfo.cmap) )
			continue ;

		dlspeed  	= pit->tfRate * 1024.0;
		fileSize 	= info.size;
		completed 	= info.transfered;
		progress 	= info.transfered * 100.0 / info.size;
		remaining   = (info.size - info.transfered) / (info.tfRate * 1024.0);

		// Estimate the completion. We need something more accurate, meaning that we need to 
		// transmit the completion info.
		//
		uint32_t chunk_size = 1024*1024 ;
 		uint32_t nb_chunks = (uint32_t)(info.size / (uint64_t)(chunk_size) ) ;
		if((info.size % (uint64_t)chunk_size) != 0)
			++nb_chunks ;
		
		uint32_t filled_chunks = pinfo.cmap.filledChunks(nb_chunks) ;

		if(filled_chunks > 1)
		{
			pinfo.progress = filled_chunks*100.0/nb_chunks ;
			completed = std::min(info.size,((uint64_t)filled_chunks)*chunk_size) ;
		}
		else
			pinfo.progress = progress ;

		addUploadItem(symbol, name, coreId, fileSize, pinfo, dlspeed, sources,  status, completed, remaining);
		ulCount++;
	  }

	  if (info.peers.size() == 0)
	  {
		symbol  	= "";
		coreId		= QString::fromStdString(info.hash);
		name    	= QString::fromUtf8(info.fname.c_str());
		sources		= tr("Unknown");

		switch(info.downloadStatus)
		{
			case FT_STATE_FAILED:
				status = tr("Failed");
				break;
			case FT_STATE_OKAY:
				status = tr("Okay");
				break;
			case FT_STATE_WAITING:
				status = tr("Waiting");
				break;
			case FT_STATE_DOWNLOADING:
				status = tr("Uploading");
				break;
		    	case FT_STATE_COMPLETE:
			default:
				status = tr("Complete");
				break;

        	}

		dlspeed  	= info.tfRate * 1024.0;
		fileSize 	= info.size;
		completed 	= info.transfered;
		progress 	= info.transfered * 100.0 / info.size;
		remaining   = (info.size - info.transfered) / (info.tfRate * 1024.0);

		FileProgressInfo pinfo ;
		pinfo.progress = progress ;
		pinfo.cmap = CompressedChunkMap() ;

		addUploadItem(symbol, name, coreId, fileSize, pinfo, dlspeed, sources,  status, completed, remaining);
		ulCount++;
	  }
	}

	ui.uploadsList->setSortingEnabled(true);
}

QString TransfersDialog::getPeerName(const std::string& id) const
{
	QString res = QString::fromStdString(rsPeers->getPeerName(id)) ;

	// This is because turtle tunnels have no name (I didn't want to bother with
	// connect mgr). In such a case their id can suitably hold for a name.
	//
	if(res == "")
		return QString::fromStdString(id) ;
	else
		return res ;
}

void TransfersDialog::cancel()
{
	QString queryWrn2;
    queryWrn2.clear();
    queryWrn2.append(tr("Are you sure that you want to cancel and delete these files?"));

    if ((QMessageBox::question(this, tr("RetroShare"),queryWrn2,QMessageBox::Ok|QMessageBox::No, QMessageBox::Ok))== QMessageBox::Ok)
    {
      for(int i = 0; i <= DLListModel->rowCount(); i++)
      {
        if(selection->isRowSelected(i, QModelIndex()))
        {
        	QVector<QString> pri;
        	pri << "Low" << "Normal" << "High" << "Auto";
        	QString priority = getPriority(i, DLListModel).trimmed();
        	std::string id = getID(i, DLListModel).toStdString();

        	if (pri.indexOf(priority) >= 0)
        	{
        		/* for file that is just in dwl queue */
        		rsFiles->clearDownload(id);
        	}
        	else
        	{
#ifdef UNUSED
				QString  qname = getFileName(i, DLListModel);
				/* XXX -> Should not have to 'trim' filename ... something wrong here..
				* but otherwise, not exact filename .... BUG
				*/
				std::string name = (qname.trimmed()).toStdString();
#endif
				rsFiles->FileCancel(id); /* hash */
        	}
        }
      }
    }
    else
    return;
}

void TransfersDialog::handleDownloadRequest(const QString& url){

    RetroShareLinkAnalyzer analyzer (url);

    if (!analyzer.isValid ())
        return;

    QVector<RetroShareLinkData> linkList;
    analyzer.getFileInformation (linkList);

    std::list<std::string> srcIds;

    for (int i = 0, n = linkList.size (); i < n; ++i)
    {
        const RetroShareLinkData& linkData = linkList[i];

        rsFiles->FileRequest (linkData.getName ().toStdString (), linkData.getHash ().toStdString (),
            linkData.getSize ().toInt (), "", 0, srcIds);
    }
}

void TransfersDialog::copyLink ()
{
    QModelIndexList lst = ui.downloadList->selectionModel ()->selectedIndexes ();
    RetroShareLinkAnalyzer analyzer;

    for (int i = 0; i < lst.count (); i++)
    {
        if ( lst[i].column() == 0 )
        {
            QModelIndex & ind = lst[i];
            QString fhash= ind.model ()->data (ind.model ()->index (ind.row (), ID )).toString() ;
            QString fsize= ind.model ()->data (ind.model ()->index (ind.row (), SIZE)).toString() ;
            QString fname= ind.model ()->data (ind.model ()->index (ind.row (), NAME)).toString() ;

            analyzer.setRetroShareLink (fname, fsize, fhash);
        }
    }

    QClipboard *clipboard = QApplication::clipboard();
    clipboard->setText(analyzer.getRetroShareLink ());
}

void TransfersDialog::showDetailsDialog()
{
    static DetailsDialog *detailsdlg = new DetailsDialog();
    
    QModelIndexList lst = ui.downloadList->selectionModel ()->selectedIndexes ();
    RetroShareLinkAnalyzer analyzer;

    for (int i = 0; i < lst.count (); i++)
    {
        if (lst[i].column () == 0)
        {
            QModelIndex& ind = lst[i];
            QString fhash = ind.model ()->data (ind.model ()->index (ind.row (), ID )).toString() ;
            QString fsize = ind.model ()->data (ind.model ()->index (ind.row (), SIZE)).toString() ;
            QString fname = ind.model ()->data (ind.model ()->index (ind.row (), NAME)).toString() ;
            QString fstatus = ind.model ()->data (ind.model ()->index (ind.row (), STATUS)).toString() ;
            QString fpriority = ind.model ()->data (ind.model ()->index (ind.row (), PRIORITY)).toString() ;
            QString fsources= ind.model ()->data (ind.model ()->index (ind.row (), SOURCES)).toString() ;
            
            qulonglong filesize = ind.model ()->data (ind.model ()->index (ind.row (), SIZE)).toULongLong() ;
            double fdatarate = ind.model ()->data (ind.model ()->index (ind.row (), DLSPEED)).toDouble() ;            
            qulonglong fcompleted = ind.model ()->data (ind.model ()->index (ind.row (), COMPLETED)).toULongLong() ;
            qulonglong fremaining = ind.model ()->data (ind.model ()->index (ind.row (), REMAINING)).toULongLong() ;
            
            // Set Details.. Window Title
            detailsdlg->setWindowTitle(tr("Details:") + fname);

            // General GroupBox
            detailsdlg->setHash(fhash);
            detailsdlg->setFileName(fname);
            detailsdlg->setSize(filesize);
            detailsdlg->setStatus(fstatus);
            detailsdlg->setPriority(fpriority);
            detailsdlg->setType(QFileInfo(fname).suffix());
            
            // Transfer GroupBox
            detailsdlg->setSources(fsources);
            detailsdlg->setDatarate(fdatarate);
            detailsdlg->setCompleted(fcompleted);
            detailsdlg->setRemaining(fremaining);
            
            // retroshare link(s) Tab
            analyzer.setRetroShareLink (fname, fsize, fhash);
            detailsdlg->setLink(analyzer.getRetroShareLink ());
            
            detailsdlg->show();
            break;
        }
    }
}

// display properties of selected items
/*void DownloadingTorrents::propertiesSelection(){
  QModelIndexList selectedIndexes = downloadList->selectionModel()->selectedIndexes();
  foreach(const QModelIndex &index, selectedIndexes){
    if(index.column() == NAME){
      showProperties(index);
    }
  }
}*/



void TransfersDialog::pasteLink()
{
    QClipboard *clipboard = QApplication::clipboard();
    RetroShareLinkAnalyzer analyzer (clipboard->text ());

    if (!analyzer.isValid ())
        return;

    QVector<RetroShareLinkData> linkList;
    analyzer.getFileInformation (linkList);

    std::list<std::string> srcIds;

    for (int i = 0, n = linkList.size (); i < n; ++i)
    {
        const RetroShareLinkData& linkData = linkList[i];
        //downloadFileRequested(linkData.getName (), linkData.getSize ().toInt (),
        //    linkData.getHash (), "", -1, -1, -1, -1);
        rsFiles->FileRequest (linkData.getName ().toStdString (), linkData.getHash ().toStdString (),
            linkData.getSize().toULongLong(), "", RS_FILE_HINTS_NETWORK_WIDE, srcIds);
    }
}

void TransfersDialog::getIdOfSelectedItems(std::set<QStandardItem *>& items)
{
	items.clear();

	int i, imax = DLListModel->rowCount();
	for (i = 0; i < imax; i++) {
		bool isParentSelected = false;
		bool isChildSelected = false;

		QStandardItem *parent = DLListModel->item(i);
		if (!parent) continue;
		QModelIndex pindex = parent->index();
		if (selection->isSelected(pindex)) {
			isParentSelected = true;
		} else {
			int j, jmax = parent->rowCount();
			for (j = 0; j < jmax && !isChildSelected; j++) {
				QStandardItem *child = parent->child(j);
				if (!child) continue;
				QModelIndex cindex = child->index();
				if (selection->isSelected(cindex)) {
					isChildSelected = true;
				}
			}
		}

		/* if transfered file or it's peers are selected control it*/
		if (isParentSelected || isChildSelected) {
			QStandardItem *id = DLListModel->item(i, ID);
			items.insert(id);
		}
	}
}

bool TransfersDialog::controlTransferFile(uint32_t flags)
{
	bool result = true;

	std::set<QStandardItem *> items;
	std::set<QStandardItem *>::iterator it;
	getIdOfSelectedItems(items);
	for (it = items.begin(); it != items.end(); it ++) {
		result &= rsFiles->FileControl((*it)->data(Qt::DisplayRole).toString().toStdString(), flags);
	}

	return result;
}

void TransfersDialog::pauseFileTransfer()
{
	if (!controlTransferFile(RS_FILE_CTRL_PAUSE))
	{
		std::cerr << "pauseFileTransfer(): can't pause file transfer" << std::endl;
	}
}

void TransfersDialog::resumeFileTransfer()
{
	if (!controlTransferFile(RS_FILE_CTRL_START))
	{
		std::cerr << "resumeFileTransfer(): can't resume file transfer" << std::endl;
	}
}

void TransfersDialog::openFolderTransfer()
{
	FileInfo info;

	std::set<QStandardItem *> items;
	std::set<QStandardItem *>::iterator it;
	getIdOfSelectedItems(items);
	for (it = items.begin(); it != items.end(); it ++) {
		if (!rsFiles->FileDetails((*it)->data(Qt::DisplayRole).toString().toStdString(), RS_FILE_HINTS_DOWNLOAD, info)) continue;
		break;
	}

	/* make path for downloaded or downloading files */
	QFileInfo qinfo;
	std::string path;
	if (info.downloadStatus == FT_STATE_COMPLETE) {
		path = info.path;
	} else {
		path = rsFiles->getPartialsDirectory();
	}

	/* open folder with a suitable application */
	qinfo.setFile(path.c_str());
	if (qinfo.exists() && qinfo.isDir()) {
		if (!QDesktopServices::openUrl(QUrl::fromLocalFile(qinfo.absoluteFilePath()))) {
			std::cerr << "openFolderTransfer(): can't open folder " << path << std::endl;
		}
	}
}

void TransfersDialog::previewTransfer()
{
	FileInfo info;

	std::set<QStandardItem *> items;
	std::set<QStandardItem *>::iterator it;
	getIdOfSelectedItems(items);
	for (it = items.begin(); it != items.end(); it ++) {
		if (!rsFiles->FileDetails((*it)->data(Qt::DisplayRole).toString().toStdString(), RS_FILE_HINTS_DOWNLOAD, info)) continue;
		break;
	}

	size_t pos = info.fname.find_last_of('.');
	if (pos == -1) return;	/* can't identify type of file */

	/* check if the file is a media file */
	if (!misc::isPreviewable(info.fname.substr(pos + 1).c_str())) return;

	/* make path for downloaded or downloading files */
	bool complete = false;
	std::string path;
	if (info.downloadStatus == FT_STATE_COMPLETE) {
		path = info.path + "/" + info.fname;
		complete = true;
	} else {
		path = rsFiles->getPartialsDirectory() + "/" + info.hash;
	}

	/* open or preview them with a suitable application */
	QFileInfo qinfo;
	if (complete) {
		qinfo.setFile(QString::fromStdString(path));
		if (qinfo.exists()) {
			if (!QDesktopServices::openUrl(QUrl::fromLocalFile(qinfo.absoluteFilePath()))) {
				std::cerr << "previewTransfer(): can't preview file " << path << std::endl;
			}
		}
	} else {
		QString linkName = QString::fromStdString(path) +
							QString::fromStdString(info.fname.substr(info.fname.find_last_of('.')));
		if (QFile::link(QString::fromStdString(path), linkName)) {
			qinfo.setFile(linkName);
			if (qinfo.exists()) {
				if (!QDesktopServices::openUrl(QUrl::fromLocalFile(qinfo.absoluteFilePath()))) {
					std::cerr << "previewTransfer(): can't preview file " << path << std::endl;
				}
			}
			/* wait for the file to open then remove the link */
#ifdef WIN32
			Sleep(2000);
#else
			sleep(2);
#endif
			QFile::remove(linkName);
		} else {
			std::cerr << "previewTransfer(): can't create link for file " << path << std::endl;
		}
	}
}

void TransfersDialog::openTransfer()
{
	FileInfo info;

	std::set<QStandardItem *> items;
	std::set<QStandardItem *>::iterator it;
	getIdOfSelectedItems(items);
	for (it = items.begin(); it != items.end(); it ++) {
		if (!rsFiles->FileDetails((*it)->data(Qt::DisplayRole).toString().toStdString(), RS_FILE_HINTS_DOWNLOAD, info)) continue;
		break;
	}

	/* make path for downloaded or downloading files */
	std::string path;
	if (info.downloadStatus == FT_STATE_COMPLETE) {
		path = info.path + "/" + info.fname;

		/* open file with a suitable application */
		QFileInfo qinfo;
		qinfo.setFile(path.c_str());
		if (qinfo.exists()) {
			if (!QDesktopServices::openUrl(QUrl::fromLocalFile(qinfo.absoluteFilePath()))) {
				std::cerr << "openTransfer(): can't open file " << path << std::endl;
			}
		}
	} else {
		/* rise a message box for incompleted download file */
		QMessageBox::information(this, tr("Open Transfer"),
				tr("File %1 is not completed. If it is a media file, try to preview it.").arg(info.fname.c_str()));
	}
}

/* clear download or all queue - for pending dwls */
void TransfersDialog::clearQueuedDwl()
{
	std::set<QStandardItem *> items;
	std::set<QStandardItem *>::iterator it;
	getIdOfSelectedItems(items);

	for (it = items.begin(); it != items.end(); it ++) {
		std::string hash = (*it)->data(Qt::DisplayRole).toString().toStdString();
		rsFiles->clearDownload(hash);
	}
}
void TransfersDialog::clearQueue()
{
	rsFiles->clearQueue();
}

void TransfersDialog::chunkStreaming()
{
	setChunkStrategy(FileChunksInfo::CHUNK_STRATEGY_STREAMING) ;
}
void TransfersDialog::chunkRandom()
{
	setChunkStrategy(FileChunksInfo::CHUNK_STRATEGY_RANDOM) ;
}
void TransfersDialog::setChunkStrategy(FileChunksInfo::ChunkStrategy s)
{
	std::set<QStandardItem *> items;
	std::set<QStandardItem *>::iterator it;
	getIdOfSelectedItems(items);

	for (it = items.begin(); it != items.end(); it ++) {
		std::string hash = (*it)->data(Qt::DisplayRole).toString().toStdString();
		rsFiles->setChunkStrategy(hash, s);
	}
}
/* modify download priority actions */
void TransfersDialog::priorityLow()
{
	changePriority(0);
}
void TransfersDialog::priorityNormal()
{
	changePriority(1);
}
void TransfersDialog::priorityHigh()
{
	changePriority(2);
}
void TransfersDialog::priorityAuto()
{
	changePriority(3);
}

void TransfersDialog::changePriority(int priority)
{
	std::set<QStandardItem *> items;
	std::set<QStandardItem *>::iterator it;
	getIdOfSelectedItems(items);

	for (it = items.begin(); it != items.end(); it ++) {
		std::string hash = (*it)->data(Qt::DisplayRole).toString().toStdString();
		rsFiles->changePriority(hash, priority);
	}
}

void TransfersDialog::clearcompleted()
{
    	std::cerr << "TransfersDialog::clearcompleted()" << std::endl;
   	rsFiles->FileClearCompleted();
}

void TransfersDialog::rootdecorated()
{
    ui.downloadList->setRootIsDecorated(true);
}

void TransfersDialog::rootisnotdecorated()
{
    ui.downloadList->setRootIsDecorated(false);
}

void TransfersDialog::saveSortIndicatorDwl(int logicalIndex, Qt::SortOrder order)
{
	_sortColDwl = logicalIndex;;
	_sortOrderDwl = order;
}

void TransfersDialog::saveSortIndicatorUpl(int logicalIndex, Qt::SortOrder order)
{
	_sortColUpl = logicalIndex;;
	_sortOrderUpl = order;
}

void TransfersDialog::showFileDetails()
{
	std::string file_hash ;
	int nb_select = 0 ;

	std::cout << "new selection " << std::endl ;

	for(int i = 0; i <= DLListModel->rowCount(); i++) 
		if(selection->isRowSelected(i, QModelIndex())) 
		{
	        file_hash = getID(i, DLListModel).toStdString();
			  ++nb_select ;
		}
	if(nb_select != 1)
		dynamic_cast<FileTransferInfoWidget*>(ui.fileTransferInfoWidget->widget())->setFileHash("") ;
	else
		dynamic_cast<FileTransferInfoWidget*>(ui.fileTransferInfoWidget->widget())->setFileHash(file_hash) ;
	
	std::cout << "calling update " << std::endl ;
	dynamic_cast<FileTransferInfoWidget*>(ui.fileTransferInfoWidget->widget())->updateDisplay() ;
	std::cout << "done" << std::endl ;
}

double TransfersDialog::getProgress(int row, QStandardItemModel *model)
{
//	return model->data(model->index(row, PROGRESS), Qt::DisplayRole).toDouble();
return 0.0 ;
}

double TransfersDialog::getSpeed(int row, QStandardItemModel *model)
{
	return model->data(model->index(row, DLSPEED), Qt::DisplayRole).toDouble();
}

QString TransfersDialog::getFileName(int row, QStandardItemModel *model)
{
	return model->data(model->index(row, NAME), Qt::DisplayRole).toString();
}

QString TransfersDialog::getStatus(int row, QStandardItemModel *model)
{
	return model->data(model->index(row, STATUS), Qt::DisplayRole).toString();
}

QString TransfersDialog::getID(int row, QStandardItemModel *model)
{
	return model->data(model->index(row, ID), Qt::DisplayRole).toString();
}

QString TransfersDialog::getPriority(int row, QStandardItemModel *model)
{
	return model->data(model->index(row, PRIORITY), Qt::DisplayRole).toString();
}

qlonglong TransfersDialog::getFileSize(int row, QStandardItemModel *model)
{
	bool ok = false;
	return model->data(model->index(row, SIZE), Qt::DisplayRole).toULongLong(&ok);
}

qlonglong TransfersDialog::getTransfered(int row, QStandardItemModel *model)
{
	bool ok = false;
	return model->data(model->index(row, COMPLETED), Qt::DisplayRole).toULongLong(&ok);
}

qlonglong TransfersDialog::getRemainingTime(int row, QStandardItemModel *model)
{
	bool ok = false;
	return model->data(model->index(row, REMAINING), Qt::DisplayRole).toULongLong(&ok);
}
