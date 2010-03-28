/****************************************************************
 *  RShare is distributed under the following license:
 *
 *  Copyright (C) 2006, crypton
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

#include "rshare.h"
#include "SharedFilesDialog.h"
#include "settings/AddFileAssociationDialog.h"

#include "rsiface/rsiface.h"
#include "rsiface/rsmsgs.h"
#include "rsiface/rspeers.h"
#include "rsiface/rsfiles.h"

#include "util/RsAction.h"
#include "msgs/ChanMsgDialog.h"
#include "settings/rsharesettings.h"
#include "AddLinksDialog.h"
#include "SendLinkDialog.h"

#include "RetroShareLink.h"

#include <iostream>
#include <sstream>

#include <QClipboard>
#include <QDesktopServices>
#include <QContextMenuEvent>
#include <QMenu>
#include <QCursor>
#include <QPoint>
#include <QMouseEvent>
#include <QPixmap>
#include <QHeaderView>
#include <QTimer>
#include <QMovie>
#include <QLabel>

#include <QMessageBox>
#include <QProcess>

/* Images for context menu icons */
#define IMAGE_DOWNLOAD       ":/images/download16.png"
#define IMAGE_PLAY           ":/images/start.png"
#define IMAGE_HASH_BUSY      ":/images/settings.png"
#define IMAGE_HASH_DONE      ":/images/accepted16.png"
#define IMAGE_MSG            ":/images/message-mail.png"
#define IMAGE_ATTACHMENT     ":/images/attachment.png"
#define IMAGE_FRIEND         ":/images/peers_16x16.png"
#define IMAGE_PROGRESS       ":/images/browse-looking.gif"
#define IMAGE_COPYLINK       ":/images/copyrslink.png"
#define IMAGE_OPENFOLDER        ":/images/folderopen.png"
#define IMAGE_OPENFILE		 ":/images/fileopen.png"

const QString Image_AddNewAssotiationForFile = ":/images/kcmsystem24.png";


/** Constructor */
SharedFilesDialog::SharedFilesDialog(QWidget *parent)
: RsAutoUpdatePage(1000,parent)
{
  /* Invoke the Qt Designer generated object setup routine */
  ui.setupUi(this);



  connect(ui.checkButton, SIGNAL(clicked()), this, SLOT(forceCheck()));

  connect(ui.localButton, SIGNAL(toggled(bool)), this, SLOT(showFrame(bool)));
  connect(ui.remoteButton, SIGNAL(toggled(bool)), this, SLOT(showFrameRemote(bool)));
  connect(ui.splittedButton, SIGNAL(toggled(bool)), this, SLOT(showFrameSplitted(bool)));


  connect( ui.localDirTreeView, SIGNAL( customContextMenuRequested( QPoint ) ),
           this,            SLOT( sharedDirTreeWidgetContextMenu( QPoint ) ) );

  connect( ui.remoteDirTreeView, SIGNAL( customContextMenuRequested( QPoint ) ), this, SLOT( shareddirtreeviewCostumPopupMenu( QPoint ) ) );

  connect( ui.remoteDirTreeView, SIGNAL( doubleClicked(const QModelIndex&)), this, SLOT( downloadRemoteSelected()));
  connect( ui.downloadButton, SIGNAL( clicked()), this, SLOT( downloadRemoteSelected()));

	connect(ui.indicatorCBox, SIGNAL(currentIndexChanged(int)), this, SLOT(indicatorChanged(int)));

/*
  connect( ui.remoteDirTreeView, SIGNAL( itemExpanded( QTreeWidgetItem * ) ),
	this, SLOT( checkForLocalDirRequest( QTreeWidgetItem * ) ) );

  connect( ui.localDirTreeWidget, SIGNAL( itemExpanded( QTreeWidgetItem * ) ),
	this, SLOT( checkForRemoteDirRequest( QTreeWidgetItem * ) ) );
*/


  model = new RemoteDirModel(true);
  localModel = new RemoteDirModel(false);
  ui.remoteDirTreeView->setModel(model);
  ui.localDirTreeView->setModel(localModel);

  ui.remoteDirTreeView->setColumnHidden(3,true) ;
  ui.remoteDirTreeView->setColumnHidden(4,true) ;
  ui.localDirTreeView->setColumnHidden(4,true) ;

  connect( ui.remoteDirTreeView, SIGNAL( collapsed(const QModelIndex & ) ),
  	model, SLOT(  collapsed(const QModelIndex & ) ) );
  connect( ui.remoteDirTreeView, SIGNAL( expanded(const QModelIndex & ) ),
  	model, SLOT(  expanded(const QModelIndex & ) ) );

  connect( ui.localDirTreeView, SIGNAL( collapsed(const QModelIndex & ) ),
  	localModel, SLOT(  collapsed(const QModelIndex & ) ) );
  connect( ui.localDirTreeView, SIGNAL( expanded(const QModelIndex & ) ),
  	localModel, SLOT(  expanded(const QModelIndex & ) ) );


  /* Set header resize modes and initial section sizes  */
	QHeaderView * l_header = ui.localDirTreeView->header () ;
//	l_header->setResizeMode (0, QHeaderView::Interactive);
//	l_header->setResizeMode (1, QHeaderView::Fixed);
//	l_header->setResizeMode (2, QHeaderView::Interactive);
//	l_header->setResizeMode (3, QHeaderView::Interactive);
//	l_header->setResizeMode (4, QHeaderView::Interactive);

	l_header->resizeSection ( 0, 490 );
	l_header->resizeSection ( 1, 70 );
	l_header->resizeSection ( 2, 100 );
	l_header->resizeSection ( 3, 100 );
//	l_header->resizeSection ( 4, 100 );

	/* Set header resize modes and initial section sizes */
	QHeaderView * r_header = ui.remoteDirTreeView->header () ;

	r_header->setResizeMode (0, QHeaderView::Interactive);
	r_header->setStretchLastSection(false);

// 	r_header->setResizeMode (1, QHeaderView::Fixed);
// //	r_header->setResizeMode (2, QHeaderView::Interactive);
// 	r_header->setResizeMode (3, QHeaderView::Fixed);
// //	r_header->setResizeMode (4, QHeaderView::Interactive);


	r_header->resizeSection ( 0, 490 );
	r_header->resizeSection ( 1, 70 );
//	r_header->resizeSection ( 2, 0 );
	r_header->resizeSection ( 3, 100 );
//	r_header->resizeSection ( 4, 0 );

	l_header->setHighlightSections(false);
	r_header->setHighlightSections(false);


  /* Set Multi Selection */
  ui.remoteDirTreeView->setSelectionMode(QAbstractItemView::ExtendedSelection);
  ui.localDirTreeView->setSelectionMode(QAbstractItemView::ExtendedSelection);


  /* Hide platform specific features */
#ifdef Q_WS_WIN

#endif
  copylinklocalAct = new QAction(QIcon(IMAGE_COPYLINK), tr( "Copy retroshare Links to Clipboard" ), this );
  connect( copylinklocalAct , SIGNAL( triggered() ), this, SLOT( copyLinkLocal() ) );
  copylinklocalhtmlAct = new QAction(QIcon(IMAGE_COPYLINK), tr( "Copy retroshare Links to Clipboard (HTML)" ), this );
  connect( copylinklocalhtmlAct , SIGNAL( triggered() ), this, SLOT( copyLinkhtml() ) );
  sendlinkAct = new QAction(QIcon(IMAGE_COPYLINK), tr( "Send retroshare Links" ), this );
  connect( sendlinkAct , SIGNAL( triggered() ), this, SLOT( sendLinkTo( ) ) );
  sendhtmllinkAct = new QAction(QIcon(IMAGE_COPYLINK), tr( "Send retroshare Links (HTML)" ), this );
  connect( sendhtmllinkAct , SIGNAL( triggered() ), this, SLOT( sendHtmlLinkTo( ) ) );
  sendlinkCloudAct = new QAction(QIcon(IMAGE_COPYLINK), tr( "Send retroshare Links to Cloud" ), this );
  connect( sendlinkCloudAct , SIGNAL( triggered() ), this, SLOT( sendLinkToCloud(  ) ) );
  addlinkCloudAct = new QAction(QIcon(IMAGE_COPYLINK), tr( "Add Links to Cloud" ), this );
  connect( addlinkCloudAct , SIGNAL( triggered() ), this, SLOT( addLinkToCloud(  ) ) );
  openfileAct = new QAction(QIcon(IMAGE_OPENFILE), tr("Open File"), this);
  connect(openfileAct, SIGNAL(triggered()), this, SLOT(openfile()));
  openfolderAct = new QAction(QIcon(IMAGE_OPENFOLDER), tr("Open Folder"), this);
  connect(openfolderAct, SIGNAL(triggered()), this, SLOT(openfolder()));


}

void SharedFilesDialog::checkUpdate()
{
        /* update */
	if (rsFiles->InDirectoryCheck())
	{
		ui.checkButton->setText(tr("Checking..."));
    QMovie *movie = new QMovie(":/images/loader/16-loader.gif");
    ui.hashLabel->setMovie(movie);
    movie->start();
    movie->setSpeed(100); // 2x speed
	}
	else
	{
	  ui.checkButton->setText(tr("Check files"));
		ui.hashLabel->setPixmap(QPixmap(IMAGE_HASH_DONE));
		ui.hashLabel->setToolTip("") ;
	}

	return;
}


void SharedFilesDialog::forceCheck()
{
	rsFiles->ForceDirectoryCheck();
	return;
}


void SharedFilesDialog::shareddirtreeviewCostumPopupMenu( QPoint point )
{
      QMenu contextMnu( this );
      QMouseEvent *mevent = new QMouseEvent( QEvent::MouseButtonPress, point, Qt::RightButton, Qt::RightButton, Qt::NoModifier );

      downloadAct = new QAction(QIcon(IMAGE_DOWNLOAD), tr( "Download" ), this );
      connect( downloadAct , SIGNAL( triggered() ), this, SLOT( downloadRemoteSelected() ) );

      copyremotelinkAct = new QAction(QIcon(IMAGE_COPYLINK), tr( "Copy retroshare Link" ), this );
      connect( copyremotelinkAct , SIGNAL( triggered() ), this, SLOT( copyLinkRemote() ) );

      sendremotelinkAct = new QAction(QIcon(IMAGE_COPYLINK), tr( "Send retroshare Link" ), this );
      connect( sendremotelinkAct , SIGNAL( triggered() ), this, SLOT( sendremoteLinkTo(  ) ) );


    //  addMsgAct = new QAction( tr( "Add to Message" ), this );
    //  connect( addMsgAct , SIGNAL( triggered() ), this, SLOT( addMsgRemoteSelected() ) );


      contextMnu.clear();
      contextMnu.addAction( downloadAct);
      contextMnu.addSeparator();
      contextMnu.addAction( copyremotelinkAct);
      contextMnu.addAction( sendremotelinkAct);
   //   contextMnu.addAction( addMsgAct);
      contextMnu.exec( mevent->globalPos() );
}


void SharedFilesDialog::downloadRemoteSelected()
{
  /* call back to the model (which does all the interfacing? */

  std::cerr << "Downloading Files";
  std::cerr << std::endl;

  QItemSelectionModel *qism = ui.remoteDirTreeView->selectionModel();
  model -> downloadSelected(qism->selectedIndexes());
}

void SharedFilesDialog::copyLink (const QModelIndexList& lst, bool remote)
{
    std::vector<DirDetails> dirVec;

    if (remote)
        model->getDirDetailsFromSelect(lst, dirVec);
    else
        localModel->getDirDetailsFromSelect(lst, dirVec);

	 std::vector<RetroShareLink> urls ;

    for (int i = 0, n = dirVec.size(); i < n; ++i)
    {
        const DirDetails& details = dirVec[i];

        if (details.type == DIR_TYPE_DIR)
        {
            for (std::list<DirStub>::const_iterator cit = details.children.begin();cit != details.children.end(); ++cit)
            {
                const DirStub& dirStub = *cit;

                DirDetails details;
                uint32_t flags = DIR_FLAGS_DETAILS;
                if (remote)
                    flags |= DIR_FLAGS_REMOTE;
                else
                    flags |= DIR_FLAGS_LOCAL;

                // do not recursive copy sub dirs.
                if (!rsFiles->RequestDirDetails(dirStub.ref, details, flags) || details.type != DIR_TYPE_FILE)
                    continue;

					 RetroShareLink link(details.name.c_str(), details.count, details.hash.c_str());

					 if(link.valid())
						 urls.push_back(link) ;
            }
        }
        else
		  {
			  RetroShareLink link(details.name.c_str(), details.count, details.hash.c_str());

			  if(link.valid())
				  urls.push_back(link) ;
		  }
    }
	 RSLinkClipboard::copyLinks(urls) ;
}

void SharedFilesDialog::copyLinkRemote()
{
    QModelIndexList lst = ui.remoteDirTreeView->selectionModel ()->selectedIndexes ();
    copyLink (lst, true);
}

void SharedFilesDialog::copyLinkLocal()
{
    QModelIndexList lst = ui.localDirTreeView->selectionModel ()->selectedIndexes ();
    copyLink (lst, false);
}

void SharedFilesDialog::copyLinkhtml( )
{
    copyLinkLocal ();

    QString link = QApplication::clipboard()->text();

    QClipboard *clipboard = QApplication::clipboard();
    clipboard->setText("<a href='" +  link + "'> " + link + "</a>");

}

void SharedFilesDialog::sendremoteLinkTo()
{
    copyLinkRemote ();

    /* create a message */
    ChanMsgDialog *nMsgDialog = new ChanMsgDialog(true);

    /* fill it in
    * files are receommended already
    * just need to set peers
    */
    std::cerr << "SharedFilesDialog::sendremoteLinkTo()" << std::endl;
    nMsgDialog->newMsg();
    nMsgDialog->insertTitleText("RetroShare Link");
    nMsgDialog->insertMsgText(RSLinkClipboard::toHtml().toStdString());

    nMsgDialog->show();
}

void SharedFilesDialog::sendLinkTo()
{
    copyLinkLocal ();

    /* create a message */
    ChanMsgDialog *nMsgDialog = new ChanMsgDialog(true);


    /* fill it in
    * files are receommended already
    * just need to set peers
    */
    std::cerr << "SharedFilesDialog::sendLinkTo()" << std::endl;
    nMsgDialog->newMsg();
    nMsgDialog->insertTitleText("RetroShare Link");

    nMsgDialog->insertMsgText(RSLinkClipboard::toHtml().toStdString());

    nMsgDialog->show();
}

void SharedFilesDialog::sendHtmlLinkTo(  )
{
    copyLinkLocal ();

    /* create a message */
    ChanMsgDialog *nMsgDialog = new ChanMsgDialog(true);

    /* fill it in
    * files are receommended already
    * just need to set peers
    */
    std::cerr << "SharedFilesDialog::sendLinkTo()" << std::endl;
    nMsgDialog->newMsg();
    nMsgDialog->insertTitleText("RetroShare Link");
 //   nMsgDialog->insertHtmlText(QApplication::clipboard()->text().toStdString());// not compatible with multiple links
    nMsgDialog->insertMsgText(RSLinkClipboard::toHtml().toStdString());

    nMsgDialog->show();
}

//void SharedFilesDialog::sendLinktoChat()
//{
//	copyLinkLocal ();
//
//  static SendLinkDialog *slinkDialog = new SendLinkDialog(this);
//  
//  slinkDialog->insertMsgText(RSLinkClipboard::toHtml().toStdString());
//  slinkDialog->show();
//
//}

void SharedFilesDialog::sendLinkToCloud()
{
	copyLinkLocal ();

	AddLinksDialog *nAddLinksDialog = new AddLinksDialog(QApplication::clipboard()->text());

	nAddLinksDialog->addLinkComment();
	nAddLinksDialog->close();
}

void SharedFilesDialog::addLinkToCloud()
{
	copyLinkLocal ();

	AddLinksDialog *nAddLinksDialog = new AddLinksDialog(QApplication::clipboard()->text());

	nAddLinksDialog->show();
}

void SharedFilesDialog::playselectedfiles()
{
  /* call back to the model (which does all the interfacing? */

  std::cerr << "SharedFilesDialog::playselectedfiles()";
  std::cerr << std::endl;

  QItemSelectionModel *qism = ui.localDirTreeView->selectionModel();

  std::list<std::string> paths;
  localModel -> getFilePaths(qism->selectedIndexes(), paths);

  std::list<std::string>::iterator it;
  QStringList fullpaths;
  for(it = paths.begin(); it != paths.end(); it++)
  {
  	std::string fullpath;
  	rsFiles->ConvertSharedFilePath(*it, fullpath);
	fullpaths.push_back(QString::fromStdString(fullpath));

  	std::cerr << "Playing: " << fullpath;
  	std::cerr << std::endl;

  }

  playFiles(fullpaths);

  std::cerr << "SharedFilesDialog::playselectedfiles() Completed";
  std::cerr << std::endl;
}


//#if 0

//void SharedFilesDialog::addMsgRemoteSelected()
//{
//  /* call back to the model (which does all the interfacing? */
//
//  std::cerr << "Recommending Files";
//  std::cerr << std::endl;
//
//  QItemSelectionModel *qism = ui.remoteDirTreeView->selectionModel();
//  model -> recommendSelected(qism->selectedIndexes());
//}


//void SharedFilesDialog::recommendfile()
//{
//  /* call back to the model (which does all the interfacing? */
//
//  std::cerr << "Recommending Files";
//  std::cerr << std::endl;
//
//  QItemSelectionModel *qism = ui.localDirTreeView->selectionModel();
//  localModel -> recommendSelected(qism->selectedIndexes());
//}




//void SharedFilesDialog::recommendFileSetOnly()
//{
//  /* call back to the model (which does all the interfacing? */
//
//  std::cerr << "Recommending File Set (clearing old selection)";
//  std::cerr << std::endl;
//
//  /* clear current recommend Selection done by model */
//
//  QItemSelectionModel *qism = ui.localDirTreeView->selectionModel();
//  localModel -> recommendSelectedOnly(qism->selectedIndexes());
//}


void SharedFilesDialog::recommendFilesTo( std::string rsid )
{
	//  recommendFileSetOnly();
	//  rsicontrol -> ClearInMsg();
	//  rsicontrol -> SetInMsg(rsid, true);

	std::list<DirDetails> files_info ;

	localModel->getFileInfoFromIndexList(ui.localDirTreeView->selectionModel()->selectedIndexes(),files_info);

	if(files_info.empty())
		return ;

	/* create a message */
	ChanMsgDialog *nMsgDialog = new ChanMsgDialog(true);

	/* fill it in
	 * files are receommended already
	 * just need to set peers
	 */

	nMsgDialog->insertFileList(files_info) ;
	nMsgDialog->newMsg();
	nMsgDialog->insertTitleText("Recommendation(s)");
	nMsgDialog->insertMsgText(rsPeers->getPeerName(rsPeers->getOwnId())+" recommends " + ( (files_info.size()>1)?"a list of files":"a file")+" to you");
	nMsgDialog->addRecipient(rsid) ;

	nMsgDialog->sendMessage();
	nMsgDialog->close();
}

void SharedFilesDialog::recommendFilesToMsg( std::string rsid )
{
	std::list<DirDetails> files_info ;

	localModel->getFileInfoFromIndexList(ui.localDirTreeView->selectionModel()->selectedIndexes(),files_info);

	if(files_info.empty())
		return ;

	/* create a message */

	ChanMsgDialog *nMsgDialog = new ChanMsgDialog(true);

	nMsgDialog->insertFileList(files_info) ;
	nMsgDialog->newMsg();
	nMsgDialog->insertTitleText("Recommendation(s)");
	nMsgDialog->insertMsgText("Recommendation(s)");
	nMsgDialog->show();

	std::cout << "recommending to " << rsid << std::endl ;
	nMsgDialog->addRecipient(rsid) ;
}

//#endif


void SharedFilesDialog::openfile()
{
  /* call back to the model (which does all the interfacing? */

    std::cerr << "SharedFilesDialog::openfile" << std::endl;

	QModelIndexList qmil = ui.localDirTreeView->selectionModel()->selectedIndexes();
	localModel->openSelected(qmil, false);
}


void SharedFilesDialog::openfolder()
{
	std::cerr << "SharedFilesDialog::openfolder" << std::endl;

	QModelIndexList qmil = ui.localDirTreeView->selectionModel()->selectedIndexes();
	localModel->openSelected(qmil, true);
}

void  SharedFilesDialog::preModDirectories(bool update_local)
{

        //std::cerr << "SharedFilesDialog::preModDirectories called with update_local = " << update_local << std::endl ;
	if (update_local)
		localModel->preMods();
	else
		model->preMods();
}


void  SharedFilesDialog::postModDirectories(bool update_local)
{
        //std::cerr << "SharedFilesDialog::postModDirectories called with update_local = " << update_local << std::endl ;
	if (update_local)
	{
		localModel->postMods();
		ui.localDirTreeView->update() ;
	}
	else
	{
		model->postMods();
		ui.remoteDirTreeView->update() ;
	}

	QCoreApplication::flush();
}


void SharedFilesDialog::sharedDirTreeWidgetContextMenu( QPoint point )
{
	//=== at this moment we'll show menu only for files, not for folders
	QModelIndex midx = ui.localDirTreeView->indexAt(point);
	//if (localModel->isDir( midx ) )
	//	return;

	currentFile = localModel->data(midx, RemoteDirModel::FileNameRole).toString();

	QMenu contextMnu2( this );
	//

//	QAction* menuAction = fileAssotiationAction(currentFile) ;
	//new QAction(QIcon(IMAGE_PLAY), currentFile, this);
	//tr( "111Play File(s)" ), this );
	//      connect( openfolderAct , SIGNAL( triggered() ), this,
	//               SLOT( playselectedfiles() ) );

//	openfileAct = new QAction(QIcon(IMAGE_ATTACHMENT), tr( "Add to Recommend List" ), this );
//	connect( openfileAct , SIGNAL( triggered() ), this, SLOT( recommendfile() ) );

//	#if 0
	/* now we're going to ask who to recommend it to...
	 * First Level.
	 *
	 * Add to Recommend List.
	 * Recommend to >
	 * 	all.
	 * 	list of <people>
	 * Recommend with msg >
	 *	all.
	 * 	list of <people>
	 *
	 */

	QMenu recMenu( tr("Recommend (Automated message) To "), this );
	recMenu.setIcon(QIcon(IMAGE_ATTACHMENT));
	QMenu msgMenu( tr("Recommend in a message to "), &contextMnu2 );
	msgMenu.setIcon(QIcon(IMAGE_MSG));

	std::list<std::string> peers;
	std::list<std::string>::iterator it;

	if (!rsPeers)
	{
		/* not ready yet! */
		return;
	}

	rsPeers->getFriendList(peers);

	for(it = peers.begin(); it != peers.end(); it++)
	{
		RsPeerDetails details ;
		if(!rsPeers->getPeerDetails(*it,details))
			continue ;

		std::string name = details.name ;
		std::string location = details.location ;

		std::string nn = name + " (" + location +")" ;
		/* parents are
		 * 	recMenu
		 * 	msgMenu
		 */

		RsAction *qaf1 = new RsAction( QIcon(IMAGE_FRIEND), QString::fromStdString( nn ), &recMenu, *it );
		connect( qaf1 , SIGNAL( triggeredId( std::string ) ), this, SLOT( recommendFilesTo( std::string ) ) );
		RsAction *qaf2 = new RsAction( QIcon(IMAGE_FRIEND), QString::fromStdString( nn ), &msgMenu, *it );
		connect( qaf2 , SIGNAL( triggeredId( std::string ) ), this, SLOT( recommendFilesToMsg( std::string ) ) );

		recMenu.addAction(qaf1);
		msgMenu.addAction(qaf2);

		/* create list of ids */

	}
	//#endif

	  if(localModel->isDir( midx ) )
		  contextMnu2.addAction( openfolderAct);
	  else
	  {
//		  contextMnu2.addAction( menuAction );
		  contextMnu2.addAction( openfileAct);
	  }

	  contextMnu2.addSeparator();

	  if(!localModel->isDir( midx ) )
	  {
		  contextMnu2.addAction( copylinklocalAct);
//		  contextMnu2.addAction( copylinklocalhtmlAct);
		  contextMnu2.addAction( sendlinkAct);
//		  contextMnu2.addAction( sendhtmllinkAct);
//		  contextMnu2.addAction( sendchatlinkAct);
		  contextMnu2.addSeparator();
#ifndef RS_RELEASE_VERSION
		  contextMnu2.addAction( sendlinkCloudAct);
		  contextMnu2.addAction( addlinkCloudAct);
		  contextMnu2.addSeparator();
#endif
		  contextMnu2.addMenu( &recMenu);
		  contextMnu2.addMenu( &msgMenu);
	  }

	QMouseEvent mevent2( QEvent::MouseButtonPress, point, Qt::RightButton, Qt::RightButton, Qt::NoModifier );
	contextMnu2.exec( mevent2.globalPos() );
}

//============================================================================

QAction*
SharedFilesDialog::fileAssotiationAction(const QString fileName)
{
    QAction* result = 0;

    RshareSettings* settings = new RshareSettings();
    //QSettings* settings= new QSettings(qApp->applicationDirPath()+"/sett.ini",
    //                            QSettings::IniFormat);
    settings->beginGroup("FileAssotiations");

    QString key = AddFileAssociationDialog::cleanFileType(currentFile) ;
    if ( settings->contains(key) )
    {
        result = new QAction(QIcon(IMAGE_PLAY), tr( "Open File" ), this );
        connect( result , SIGNAL( triggered() ),
                 this, SLOT( runCommandForFile() ) );

        currentCommand = (settings->value( key )).toString();
    }
    else
    {
        result = new QAction(QIcon(Image_AddNewAssotiationForFile),
                             tr( "Set command for opening this file"), this );
        connect( result , SIGNAL( triggered() ),
                 this,    SLOT(   tryToAddNewAssotiation() ) );
    }

    delete settings;

    return result;
}

//============================================================================

void
SharedFilesDialog::runCommandForFile()
{
    QStringList tsl;
    tsl.append( currentFile );
    QProcess::execute( currentCommand, tsl);
    //QString("%1 %2").arg(currentCommand).arg(currentFile) );

//    QString tmess = "Some command(%1) should be executed here for file %2";
//    tmess = tmess.arg(currentCommand).arg(currentFile);
//    QMessageBox::warning(this, tr("RetroShare"), tmess, QMessageBox::Ok);
}

//============================================================================

void
SharedFilesDialog::tryToAddNewAssotiation()
{
    AddFileAssociationDialog afad(true, this);//'add file assotiations' dialog

    afad.setFileType(AddFileAssociationDialog::cleanFileType(currentFile));

    int ti = afad.exec();

    if (ti==QDialog::Accepted)
    {
        RshareSettings* settings = new RshareSettings();
        //QSettings settings( qApp->applicationDirPath()+"/sett.ini",
        //                    QSettings::IniFormat);
        settings->beginGroup("FileAssotiations");

        QString currType = afad.resultFileType() ;
        QString currCmd = afad.resultCommand() ;

        settings->setValue(currType, currCmd);
    }
}

//============================================================================
/**
 Toggles the Splitted, Remote and Local View on and off*/

void SharedFilesDialog::showFrame(bool show)
{
    if (show) {
        ui.localframe->setVisible(true);
        ui.remoteframe->setVisible(false);

        ui.localButton->setChecked(true);

        ui.remoteButton->setChecked(false);
        ui.splittedButton->setChecked(false);

        /* set textcolor for Channel name  */
        QString labelStr("<span style=\"font-size:10pt; font-weight:500;"
                               "color:#000000;\">%1</span>");

        ui.labeltext->setText( labelStr.arg(tr("<strong>My Shared Files</strong>")));
    }
}

void SharedFilesDialog::showFrameRemote(bool show)
{
    if (show) {
        ui.remoteframe->setVisible(true);
        ui.localframe->setVisible(false);

        ui.remoteButton->setChecked(true);
        ui.localButton->setChecked(false);
        ui.splittedButton->setChecked(false);

        ui.labeltext->setText( tr("<strong>Friends Files</strong>"));
    }
}

void SharedFilesDialog::showFrameSplitted(bool show)
{
    if (show) {
        ui.remoteframe->setVisible(true);
        ui.localframe->setVisible(true);

        ui.splittedButton->setChecked(true);

        ui.localButton->setChecked(false);
        ui.remoteButton->setChecked(false);

        ui.labeltext->setText( tr("<strong>Files</strong>"));
    }
}

void SharedFilesDialog::indicatorChanged(int index)
{
	static uint32_t correct_indicator[4] = { IND_ALWAYS,IND_LAST_DAY,IND_LAST_WEEK,IND_LAST_MONTH } ;

	model->changeAgeIndicator(correct_indicator[index]);
	localModel->changeAgeIndicator(correct_indicator[index]);

  ui.remoteDirTreeView->update(ui.remoteDirTreeView->rootIndex());
  ui.localDirTreeView->update(ui.localDirTreeView->rootIndex()) ;

  updateDisplay() ;
}

