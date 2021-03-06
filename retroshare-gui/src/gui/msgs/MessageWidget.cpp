/****************************************************************
 *  RetroShare is distributed under the following license:
 *
 *  Copyright (C) 2006 - 2011 RetroShare Team
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

#include <QMenu>
#include <QToolButton>
#include <QDateTime>
#include <QMessageBox>
#include <QPrinter>
#include <QPrintDialog>
#include <QFile>
#include <QTextStream>
#include <QTextCodec>

#include "gui/notifyqt.h"
#include "gui/RetroShareLink.h"
#include "gui/common/TagDefs.h"
#include "gui/common/PeerDefs.h"
#include "gui/common/Emoticons.h"
#include "gui/settings/rsharesettings.h"
#include "MessageComposer.h"
#include "MessageWidget.h"
#include "MessageWindow.h"
#include "util/misc.h"
#include "util/printpreview.h"
#include "util/HandleRichText.h"

#include <retroshare/rspeers.h>
#include <retroshare/rsfiles.h>
#include <retroshare/rsmsgs.h>

/* Images for context menu icons */
#define IMAGE_DOWNLOAD         ":/images/start.png"
#define IMAGE_DOWNLOADALL      ":/images/startall.png"

#define COLUMN_FILE_NAME   0
#define COLUMN_FILE_SIZE   1
#define COLUMN_FILE_HASH   2
#define COLUMN_FILE_COUNT  3

class RsHtmlMsg : public RsHtml
{
public:
	RsHtmlMsg(uint msgFlags) : RsHtml()
	{
		this->msgFlags = msgFlags;
	}

protected:
	virtual void anchorTextForImg(QDomDocument &doc, QDomElement &element, const RetroShareLink &link, QString &text)
	{
		if (link.type() == RetroShareLink::TYPE_CERTIFICATE) {
			if (msgFlags & RS_MSG_USER_REQUEST) {
				text = QApplication::translate("MessageWidget", "Confirm %1 as friend").arg(link.name());
				return;
			}
			if (msgFlags & RS_MSG_FRIEND_RECOMMENDATION) {
				text = QApplication::translate("MessageWidget", "Add %1 as friend").arg(link.name());
				return;
			}
		}

		RsHtml::anchorTextForImg(doc, element, link, text);
	}

protected:
	uint msgFlags;
};

MessageWidget *MessageWidget::openMsg(const std::string &msgId, bool window)
{
	if (msgId.empty()) {
		return NULL;
	}

	MessageInfo msgInfo;
	if (!rsMsgs->getMessage(msgId, msgInfo)) {
		std::cerr << "MessageWidget::openMsg() Couldn't find Msg" << std::endl;
		return NULL;
	}

	MessageWindow *parent = NULL;
	if (window) {
		parent = new MessageWindow;
	}
	MessageWidget *msgWidget = new MessageWidget(false, parent);
	msgWidget->isWindow = window;
    msgWidget->fill(msgId);
	if (parent) {
		parent->addWidget(msgWidget);
	}

	if (parent) {
		parent->show();
		parent->activateWindow();
	}

	return msgWidget;
}

/** Constructor */
MessageWidget::MessageWidget(bool controlled, QWidget *parent, Qt::WFlags flags)
: QWidget(parent, flags)
{
	/* Invoke the Qt Designer generated object setup routine */
	ui.setupUi(this);

	isControlled = controlled;
	isWindow = false;
	currMsgFlags = 0;

	connect(ui.msgList, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(msgfilelistWidgetCostumPopupMenu(QPoint)));
	connect(ui.expandFilesButton, SIGNAL(clicked()), this, SLOT(togglefileview()));
	connect(ui.downloadButton, SIGNAL(clicked()), this, SLOT(getallrecommended()));
	connect(ui.msgText, SIGNAL(anchorClicked(QUrl)), this, SLOT(anchorClicked(QUrl)));

	connect(NotifyQt::getInstance(), SIGNAL(messagesTagsChanged()), this, SLOT(messagesTagsChanged()));
	connect(NotifyQt::getInstance(), SIGNAL(messagesChanged()), this, SLOT(messagesChanged()));

	/* hide the Tree +/- */
	ui.msgList->setRootIsDecorated( false );
	ui.msgList->setSelectionMode( QAbstractItemView::ExtendedSelection );

	/* Set header resize modes and initial section sizes */
	QHeaderView * msglheader = ui.msgList->header () ;
	msglheader->setResizeMode (COLUMN_FILE_NAME, QHeaderView::Interactive);
	msglheader->setResizeMode (COLUMN_FILE_SIZE, QHeaderView::Interactive);
	msglheader->setResizeMode (COLUMN_FILE_HASH, QHeaderView::Interactive);

	msglheader->resizeSection (COLUMN_FILE_NAME, 200);
	msglheader->resizeSection (COLUMN_FILE_SIZE, 100);
	msglheader->resizeSection (COLUMN_FILE_HASH, 200);

	QFont font = QFont("Arial", 10, QFont::Bold);
	ui.subjectText->setFont(font);

	ui.bcclabel->setVisible(false);
	ui.bccText->setVisible(false);
	ui.cclabel->setVisible(false);
	ui.ccText->setVisible(false);

	ui.tagsLabel->setVisible(false);

	if (isControlled == false) {
		processSettings("MessageWidget", true);
	}

	ui.dateText-> setText("");

	/* Hide platform specific features */
#ifdef Q_WS_WIN

#endif
}

MessageWidget::~MessageWidget()
{
	if (isControlled == false) {
		processSettings("MessageWidget", false);
	}
}

void MessageWidget::connectAction(enumActionType actionType, QToolButton* button)
{
	switch (actionType) {
	case ACTION_REMOVE:
		connect(button, SIGNAL(clicked()), this, SLOT(remove()));
		break;
	case ACTION_REPLY:
		connect(button, SIGNAL(clicked()), this, SLOT(reply()));
		break;
	case ACTION_REPLY_ALL:
		connect(button, SIGNAL(clicked()), this, SLOT(replyAll()));
		break;
	case ACTION_FORWARD:
		connect(button, SIGNAL(clicked()), this, SLOT(forward()));
		break;
	case ACTION_PRINT:
		connect(button, SIGNAL(clicked()), this, SLOT(print()));
		break;
	case ACTION_PRINT_PREVIEW:
		connect(button, SIGNAL(clicked()), this, SLOT(printPreview()));
		break;
	case ACTION_SAVE_AS:
		connect(button, SIGNAL(clicked()), this, SLOT(saveAs()));
		break;
	}
}

void MessageWidget::connectAction(enumActionType actionType, QAction *action)
{
	switch (actionType) {
	case ACTION_REMOVE:
		connect(action, SIGNAL(triggered()), this, SLOT(remove()));
		break;
	case ACTION_REPLY:
		connect(action, SIGNAL(triggered()), this, SLOT(reply()));
		break;
	case ACTION_REPLY_ALL:
		connect(action, SIGNAL(triggered()), this, SLOT(replyAll()));
		break;
	case ACTION_FORWARD:
		connect(action, SIGNAL(triggered()), this, SLOT(forward()));
		break;
	case ACTION_PRINT:
		connect(action, SIGNAL(triggered()), this, SLOT(print()));
		break;
	case ACTION_PRINT_PREVIEW:
		connect(action, SIGNAL(triggered()), this, SLOT(printPreview()));
		break;
	case ACTION_SAVE_AS:
		connect(action, SIGNAL(triggered()), this, SLOT(saveAs()));
		break;
	}
}

void MessageWidget::processSettings(const QString &settingsGroup, bool load)
{
	Settings->beginGroup(settingsGroup);

	if (load) {
		// load settings

		// expandFiles
		bool value = Settings->value("expandFiles", true).toBool();
		ui.expandFilesButton->setChecked(value);
		ui.msgList->setVisible(value);
		togglefileview();
	} else {
		// save settings

		// expandFiles
		Settings->setValue("expandFiles", ui.expandFilesButton->isChecked());
	}

	Settings->endGroup();
}

QString MessageWidget::subject(bool noEmpty)
{
	QString subject = ui.subjectText->text();
	if (subject.isEmpty() && noEmpty) {
		return "[" + tr("No subject") + "]";
	}

	return subject;
}

void MessageWidget::msgfilelistWidgetCostumPopupMenu( QPoint /*point*/ )
{
	QMenu contextMnu(this);

	contextMnu.addAction(QIcon(IMAGE_DOWNLOAD), tr("Download"), this, SLOT(getcurrentrecommended()));
	contextMnu.addAction(QIcon(IMAGE_DOWNLOADALL), tr("Download all"), this, SLOT(getallrecommended()));

	contextMnu.exec(QCursor::pos());
}

void MessageWidget::togglefileview()
{
	/* if msg header visible -> change icon and tooltip
	* three widgets...
	*/

	if (ui.expandFilesButton->isChecked()) {
		ui.expandFilesButton->setIcon(QIcon(QString(":/images/edit_remove24.png")));
		ui.expandFilesButton->setToolTip(tr("Hide"));
	} else {
		ui.expandFilesButton->setIcon(QIcon(QString(":/images/edit_add24.png")));
		ui.expandFilesButton->setToolTip(tr("Expand"));
	}
}

/* download the recommendations... */
void MessageWidget::getcurrentrecommended()
{
	MessageInfo msgInfo;
	if (rsMsgs->getMessage(currMsgId, msgInfo) == false) {
		return;
	}

	std::list<std::string> srcIds;
	srcIds.push_back(msgInfo.srcId);

	QModelIndexList list = ui.msgList->selectionModel()->selectedIndexes();

	std::map<int,FileInfo> files ;

	for (QModelIndexList::const_iterator it(list.begin());it!=list.end();++it) {
		FileInfo& fi(files[it->row()]) ;

		switch (it->column()) {
		case COLUMN_FILE_NAME:
			fi.fname = it->data().toString().toUtf8().constData();
			break ;
		case COLUMN_FILE_SIZE:
			fi.size = it->data().toULongLong() ;
			break ;
		case COLUMN_FILE_HASH:
			fi.hash = it->data().toString().toStdString() ;
			break ;
		}
	}

	for(std::map<int,FileInfo>::const_iterator it(files.begin());it!=files.end();++it) {
		const FileInfo& fi(it->second) ;
		std::cout << "Requesting file " << fi.fname << ", size=" << fi.size << ", hash=" << fi.hash << std::endl ;

		if (rsFiles->FileRequest(fi.fname, fi.hash, fi.size, "", RS_FILE_HINTS_NETWORK_WIDE, srcIds) == false) {
			QMessageBox mb(QObject::tr("File Request canceled"), QObject::tr("The following has not been added to your download list, because you already have it:\n    ") + QString::fromUtf8(fi.fname.c_str()), QMessageBox::Critical, QMessageBox::Ok, 0, 0);
			mb.setWindowIcon(QIcon(QString::fromUtf8(":/images/rstray3.png")));
			mb.exec();
		}
	}
}

void MessageWidget::getallrecommended()
{
	/* get Message */
	MessageInfo msgInfo;
	if (rsMsgs->getMessage(currMsgId, msgInfo) == false) {
		return;
	}

	const std::list<FileInfo> &recList = msgInfo.files;
	std::list<FileInfo>::const_iterator it;

	/* do the requests */
	for(it = recList.begin(); it != recList.end(); it++) {
		std::cerr << "MessageWidget::getallrecommended() Calling File Request" << std::endl;
		std::list<std::string> srcIds;
		srcIds.push_back(msgInfo.srcId);
		rsFiles->FileRequest(it->fname, it->hash, it->size, "", RS_FILE_HINTS_NETWORK_WIDE, srcIds);
	}
}

void MessageWidget::messagesTagsChanged()
{
    showTagLabels();
}

void MessageWidget::messagesChanged()
{
	if (isControlled) {
		/* processed by MessagesDialog */
		return;
	}

	/* test Message */
	MessageInfo msgInfo;
	if (rsMsgs->getMessage(currMsgId, msgInfo) == false) {
		/* messages was removed */
		if (isWindow) {
			window()->close();
		} else {
			deleteLater();
		}
	}
}

void MessageWidget::clearTagLabels()
{
	/* clear all tags */
	while (tagLabels.size()) {
		delete tagLabels.front();
		tagLabels.pop_front();
	}
	while (ui.tagLayout->count()) {
		delete ui.tagLayout->takeAt(0);
	}

	ui.tagsLabel->setVisible(false);
}

void MessageWidget::showTagLabels()
{
	clearTagLabels();

	if (currMsgId.empty()) {
		return;
	}

	MsgTagInfo tagInfo;
	rsMsgs->getMessageTag(currMsgId, tagInfo);

	if (tagInfo.tagIds.empty() == false) {
		ui.tagsLabel->setVisible(true);

		MsgTagType Tags;
		rsMsgs->getMessageTagTypes(Tags);

		std::map<uint32_t, std::pair<std::string, uint32_t> >::iterator Tag;
		for (std::list<uint32_t>::iterator tagId = tagInfo.tagIds.begin(); tagId != tagInfo.tagIds.end(); tagId++) {
			Tag = Tags.types.find(*tagId);
			if (Tag != Tags.types.end()) {
				QLabel *tagLabel = new QLabel(TagDefs::name(Tag->first, Tag->second.first), this);
				tagLabel->setMaximumHeight(16);
				tagLabel->setStyleSheet(TagDefs::labelStyleSheet(Tag->second.second));
				tagLabels.push_back(tagLabel);
				ui.tagLayout->addWidget(tagLabel);
				ui.tagLayout->addSpacing(3);
			}
		}
		ui.tagLayout->addStretch();
	} else {
		ui.tagsLabel->setVisible(false);
	}
}

void MessageWidget::fill(const std::string &msgId)
{
	if (currMsgId == msgId) {
		// message doesn't changed
		return;
	}

	currMsgId = msgId;

	if (currMsgId.empty()) {
		/* blank it */
		ui.dateText-> setText("");
		ui.toText->setText("");
		ui.fromText->setText("");
		ui.filesText->setText("");

		ui.cclabel->setVisible(false);
		ui.ccText->setVisible(false);
		ui.ccText->clear();

		ui.bcclabel->setVisible(false);
		ui.bccText->setVisible(false);
		ui.bccText->clear();

		ui.subjectText->setText("");
		ui.msgList->clear();
		ui.msgText->clear();

		clearTagLabels();

		currMsgFlags = 0;

		return;
	}

	clearTagLabels();

	MessageInfo msgInfo;
	if (rsMsgs->getMessage(currMsgId, msgInfo) == false) {
		std::cerr << "MessageWidget::fill() Couldn't find Msg" << std::endl;
		return;
	}

	const std::list<FileInfo> &recList = msgInfo.files;
	std::list<FileInfo>::const_iterator it;

	ui.msgList->clear();

	QList<QTreeWidgetItem*> items;
	for (it = recList.begin(); it != recList.end(); it++) {
		QTreeWidgetItem *item = new QTreeWidgetItem;
		item->setText(COLUMN_FILE_NAME, QString::fromUtf8(it->fname.c_str()));
		item->setText(COLUMN_FILE_SIZE, QString::number(it->size));
		item->setText(COLUMN_FILE_HASH, QString::fromStdString(it->hash));

		/* add to the list */
		items.append(item);
	}

	/* add the items in! */
	ui.msgList->insertTopLevelItems(0, items);

	/* iterate through the sources */
	std::list<std::string>::const_iterator pit;

	RetroShareLink link;
	QString text;

	for(pit = msgInfo.msgto.begin(); pit != msgInfo.msgto.end(); pit++) {
		if (link.createMessage(*pit, "")) {
			text += link.toHtml() + "   ";
		}
	}
	ui.toText->setText(text);

	if (msgInfo.msgcc.size() > 0) {
		ui.cclabel->setVisible(true);
		ui.ccText->setVisible(true);

		text.clear();
		for(pit = msgInfo.msgcc.begin(); pit != msgInfo.msgcc.end(); pit++) {
			if (link.createMessage(*pit, "")) {
				text += link.toHtml() + "   ";
			}
		}
		ui.ccText->setText(text);
	} else {
		ui.cclabel->setVisible(false);
		ui.ccText->setVisible(false);
		ui.ccText->clear();
	}

	if (msgInfo.msgbcc.size() > 0) {
		ui.bcclabel->setVisible(true);
		ui.bccText->setVisible(true);

		text.clear();
		for(pit = msgInfo.msgbcc.begin(); pit != msgInfo.msgbcc.end(); pit++) {
			if (link.createMessage(*pit, "")) {
				text += link.toHtml() + "   ";
			}
		}
		ui.bccText->setText(text);
	} else {
		ui.bcclabel->setVisible(false);
		ui.bccText->setVisible(false);
		ui.bccText->clear();
	}

	{
		QDateTime qtime;
		qtime.setTime_t(msgInfo.ts);
		QString timestamp = qtime.toString("dd.MM.yyyy hh:mm:ss");
		ui.dateText->setText(timestamp);
	}

	std::string ownId = rsPeers->getOwnId();

	std::string srcId;
	if ((msgInfo.msgflags & RS_MSG_BOXMASK) == RS_MSG_OUTBOX) {
		// outgoing message are from me
		srcId = ownId;
	} else {
		srcId = msgInfo.srcId;
	}
	link.createMessage(srcId, "");

	if ((msgInfo.msgflags & RS_MSG_SYSTEM) && msgInfo.srcId == ownId) {
		ui.fromText->setText("RetroShare");
	} else {
		ui.fromText->setText(link.toHtml());
		ui.fromText->setToolTip(PeerDefs::rsidFromId(srcId));
	}

	ui.subjectText->setText(QString::fromStdWString(msgInfo.title));

	text = RsHtmlMsg(msgInfo.msgflags).formatText(ui.msgText->document(), QString::fromStdWString(msgInfo.msg), RSHTML_FORMATTEXT_EMBED_SMILEYS | RSHTML_FORMATTEXT_EMBED_LINKS | RSHTML_FORMATTEXT_REPLACE_LINKS);
	ui.msgText->setHtml(text);

	ui.filesText->setText(QString("(%1 %2)").arg(msgInfo.count).arg(msgInfo.count == 1 ? tr("File") : tr("Files")));

	showTagLabels();

	currMsgFlags = msgInfo.msgflags;
}

void MessageWidget::remove()
{
	MessageInfo msgInfo;
	if (rsMsgs->getMessage(currMsgId, msgInfo) == false) {
		std::cerr << "MessageWidget::fill() Couldn't find Msg" << std::endl;
		return;
	}

	bool deleteReal = false;
	if (msgInfo.msgflags & RS_MSG_TRASH) {
		deleteReal = true;
	} else {
		if (QApplication::keyboardModifiers() & Qt::ShiftModifier) {
			deleteReal = true;
		}
	}

	if (deleteReal) {
		rsMsgs->MessageDelete(currMsgId);
	} else {
		rsMsgs->MessageToTrash(currMsgId, true);
	}

	if (isWindow) {
		window()->close();
	} else {
		deleteLater();
	}
}

void MessageWidget::print()
{
#ifndef QT_NO_PRINTER
	QPrinter printer(QPrinter::HighResolution);
	printer.setFullPage(true);
	QPrintDialog *dlg = new QPrintDialog(&printer, this);
	if (ui.msgText->textCursor().hasSelection())
		dlg->addEnabledOption(QAbstractPrintDialog::PrintSelection);
	dlg->setWindowTitle(tr("Print Document"));
	if (dlg->exec() == QDialog::Accepted) {
		ui.msgText->print(&printer);
	}
	delete dlg;
#endif
}

void MessageWidget::printPreview()
{
	PrintPreview *preview = new PrintPreview(ui.msgText->document(), this);
	preview->setWindowModality(Qt::WindowModal);
	preview->setAttribute(Qt::WA_DeleteOnClose);
	preview->show();

	/* window will destroy itself! */
}

void MessageWidget::saveAs()
{
	QString filename;
	if (misc::getSaveFileName(window(), RshareSettings::LASTDIR_MESSAGES, tr("Save as..."), tr("HTML-Files (*.htm *.html);;All Files (*)"), filename)) {
		QFile file(filename);
		if (!file.open(QFile::WriteOnly))
			return;
		QTextStream ts(&file);
		ts.setCodec(QTextCodec::codecForName("UTF-8"));
		ts << ui.msgText->document()->toHtml("UTF-8");
		ui.msgText->document()->setModified(false);
	}
}

void MessageWidget::reply()
{
	/* put msg on msgBoard, and switch to it. */

	if (currMsgId.empty()) {
		return;
	}

	MessageComposer *msgComposer = MessageComposer::replyMsg(currMsgId, false);
	if (msgComposer == NULL) {
		return;
	}

	msgComposer->show();
	msgComposer->activateWindow();

	/* window will destroy itself! */
}

void MessageWidget::replyAll()
{
	/* put msg on msgBoard, and switch to it. */

	if (currMsgId.empty()) {
		return;
	}

	MessageComposer *msgComposer = MessageComposer::replyMsg(currMsgId, true);
	if (msgComposer == NULL) {
		return;
	}

	msgComposer->show();
	msgComposer->activateWindow();

	/* window will destroy itself! */
}

void MessageWidget::forward()
{
	/* put msg on msgBoard, and switch to it. */

	if (currMsgId.empty()) {
		return;
	}

	MessageComposer *msgComposer = MessageComposer::forwardMsg(currMsgId);
	if (msgComposer == NULL) {
		return;
	}

	msgComposer->show();
	msgComposer->activateWindow();

	/* window will destroy itself! */
}

void MessageWidget::anchorClicked(const QUrl &url)
{
	RetroShareLink link(url);

	if (link.valid() == false) {
		return;
	}

	if (link.type() == RetroShareLink::TYPE_CERTIFICATE && currMsgFlags & RS_MSG_USER_REQUEST) {
		link.setSubType(RSLINK_SUBTYPE_CERTIFICATE_USER_REQUEST);
	}

	QList<RetroShareLink> links;
	links.append(link);
	RetroShareLink::process(links);
}
