/****************************************************************
 *  RetroShare is distributed under the following license:
 *
 *  Copyright (C) 2006 - 2010  The RetroShare Team
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

#include <QMessageBox>
#include <QDateTime>
#include <QMenu>
#include <QClipboard>
#include <QTextDocument>
#include <QTextEdit>
#include <QClipboard>
#include <QKeyEvent>
#include <QThread>

#include "ImHistoryBrowser.h"
#include "IMHistoryItemDelegate.h"
#include "IMHistoryItemPainter.h"

#include "rshare.h"
#include "gui/settings/rsharesettings.h"
#include "gui/notifyqt.h"

#define ROLE_HIID      Qt::UserRole
#define ROLE_PLAINTEXT Qt::UserRole + 1
#define ROLE_OFFLINE   Qt::UserRole + 2

ImHistoryBrowserCreateItemsThread::ImHistoryBrowserCreateItemsThread(ImHistoryBrowser *parent, IMHistoryKeeper &histKeeper)
    : QThread(parent), m_historyKeeper(histKeeper)
{
    m_historyBrowser = parent;
    stopped = false;
}

ImHistoryBrowserCreateItemsThread::~ImHistoryBrowserCreateItemsThread()
{
    // remove all items (when items are available, the thread was terminated)
    QList<QListWidgetItem*>::iterator it;
    for (it = m_items.begin(); it != m_items.end(); it++) {
        delete(*it);
    }

    m_items.clear();
}

void ImHistoryBrowserCreateItemsThread::stop()
{
    disconnect();
    stopped = true;
    wait();
}

void ImHistoryBrowserCreateItemsThread::run()
{
    QList<IMHistoryItem> historyItems;
    m_historyKeeper.getMessages(historyItems, 0);

    int count = historyItems.count();
    int current = 0;

    foreach(IMHistoryItem item, historyItems) {
        if (stopped) {
            break;
        }
        QListWidgetItem *itemWidget = m_historyBrowser->createItem(item);
        if (itemWidget) {
            m_items.push_back(itemWidget);
            emit progress(++current, count);
        }
    }
}

/** Default constructor */
ImHistoryBrowser::ImHistoryBrowser(const std::string &peerId, IMHistoryKeeper &histKeeper, QTextEdit *edit, QWidget *parent, Qt::WFlags flags)
  : QDialog(parent, flags), historyKeeper(histKeeper)
{
    /* Invoke Qt Designer generated QObject setup routine */
    ui.setupUi(this);

    m_peerId = peerId;
    m_isPrivateChat = !m_peerId.empty();
    textEdit = edit;

    connect(&historyKeeper, SIGNAL(historyAdd(IMHistoryItem)), this, SLOT(historyAdd(IMHistoryItem)));
    connect(&historyKeeper, SIGNAL(historyRemove(IMHistoryItem)), this, SLOT(historyRemove(IMHistoryItem)));
    connect(&historyKeeper, SIGNAL(historyClear()), this, SLOT(historyClear()));

    connect(ui.clearFilterButton, SIGNAL(clicked()), this, SLOT(clearFilter()));
    connect(ui.filterPatternLineEdit, SIGNAL(textChanged(const QString &)), this, SLOT(filterRegExpChanged()));

    connect(ui.copyButton, SIGNAL(clicked()), SLOT(copyMessage()));
    connect(ui.removeButton, SIGNAL(clicked()), SLOT(removeMessages()));

    connect(ui.listWidget, SIGNAL(itemSelectionChanged()), this, SLOT(itemSelectionChanged()));
    connect(ui.listWidget, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(customContextMenuRequested(QPoint)));

    connect(NotifyQt::getInstance(), SIGNAL(privateChatChanged(int,int)), this, SLOT(privateChatChanged(int,int)));

    ui.clearFilterButton->hide();

    // embed smileys ?
    if (m_isPrivateChat) {
        embedSmileys = Settings->valueFromGroup("Chat", "Emoteicons_PrivatChat", true).toBool();
    } else {
        embedSmileys = Settings->valueFromGroup("Chat", "Emoteicons_GroupChat", true).toBool();
    }

    style.setStyleFromSettings(ChatStyle::TYPE_HISTORY);

    ui.listWidget->setItemDelegate(new IMHistoryItemDelegate);

    // call once
    privateChatChanged(NOTIFY_LIST_PRIVATE_OUTGOING_CHAT, NOTIFY_TYPE_ADD);

    QByteArray geometry = Settings->valueFromGroup("HistorieBrowser", "Geometry", QByteArray()).toByteArray();
    if (geometry.isEmpty() == false) {
        restoreGeometry(geometry);
    }

    // dummy call for set buttons
    itemSelectionChanged();

    ui.listWidget->installEventFilter(this);

    m_createThread = new ImHistoryBrowserCreateItemsThread(this, historyKeeper);
    connect(m_createThread, SIGNAL(finished()), this, SLOT(createThreadFinished()));
    connect(m_createThread, SIGNAL(progress(int,int)), this, SLOT(createThreadProgress(int,int)));
    m_createThread->start();
}

ImHistoryBrowser::~ImHistoryBrowser()
{
    Settings->setValueToGroup("HistorieBrowser", "Geometry", saveGeometry());

    if (m_createThread) {
        m_createThread->stop();
        delete(m_createThread);
        m_createThread = NULL;
    }
}

void ImHistoryBrowser::createThreadFinished()
{
    if (m_createThread == sender()) {
        ui.progressBar->setVisible(false);

        if (!m_createThread->wasStopped()) {
            // append created items
            QList<QListWidgetItem*>::iterator it;
            for (it = m_createThread->m_items.begin(); it != m_createThread->m_items.end(); it++) {
                ui.listWidget->addItem(*it);
            }

            // clear list
            m_createThread->m_items.clear();

            filterRegExpChanged();

            // dummy call for set buttons
            itemSelectionChanged();

            m_createThread->deleteLater();
            m_createThread = NULL;

            QList<IMHistoryItem>::iterator histIt;
            for (histIt = m_itemsAddedOnLoad.begin(); histIt != m_itemsAddedOnLoad.end(); histIt++) {
                historyAdd(*histIt);
            }
            m_itemsAddedOnLoad.clear();
        }
    }
}

void ImHistoryBrowser::createThreadProgress(int current, int count)
{
    if (count) {
        ui.progressBar->setValue(current * ui.progressBar->maximum() / count);
    }
}

bool ImHistoryBrowser::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == ui.listWidget) {
        if (event->type() == QEvent::KeyPress) {
            QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
            if (keyEvent && keyEvent->key() == Qt::Key_Delete) {
                // Delete pressed
                removeMessages();
                return true; // eat event
            }
        }
    }
    // pass the event on to the parent class
    return QDialog::eventFilter(obj, event);
}

void ImHistoryBrowser::historyAdd(IMHistoryItem item)
{
    if (m_createThread) {
        // create later
        m_itemsAddedOnLoad.push_back(item);
        return;
    }

    QListWidgetItem *itemWidget = createItem(item);
    if (itemWidget) {
        ui.listWidget->addItem(itemWidget);
        filterItems(itemWidget);
    }
}

void ImHistoryBrowser::historyRemove(IMHistoryItem item)
{
    int count = ui.listWidget->count();
    for (int i = 0; i < count; i++) {
        QListWidgetItem *itemWidget = ui.listWidget->item(i);
        if (itemWidget->data(ROLE_HIID).toString().toInt() == item.hiid) {
            delete(ui.listWidget->takeItem(i));
            break;
        }
    }
}

void ImHistoryBrowser::historyClear()
{
    ui.listWidget->clear();
}

void ImHistoryBrowser::fillItem(QListWidgetItem *itemWidget, IMHistoryItem &item)
{
    unsigned int formatFlag = CHAT_FORMATMSG_EMBED_LINKS;

    if (embedSmileys) {
        formatFlag |= CHAT_FORMATMSG_EMBED_SMILEYS;
    }

    std::list<ChatInfo>::iterator offineChatIt;
    for(offineChatIt = m_savedOfflineChat.begin(); offineChatIt != m_savedOfflineChat.end(); offineChatIt++) {
        /* are they public? */
        if ((offineChatIt->chatflags & RS_CHAT_PRIVATE) == 0) {
            /* this should not happen */
            continue;
        }

        QDateTime sendTime = QDateTime::fromTime_t(offineChatIt->sendTime);
        QString message = QString::fromStdWString(offineChatIt->msg);

        if (IMHistoryKeeper::compareItem(item, false, offineChatIt->rsid, sendTime, message)) {
            break;
        }
    }

    ChatStyle::enumFormatMessage type;
    if (offineChatIt == m_savedOfflineChat.end()) {
        if (item.incoming) {
            type = ChatStyle::FORMATMSG_INCOMING;
        } else {
            type = ChatStyle::FORMATMSG_OUTGOING;
        }
    } else {
        type = ChatStyle::FORMATMSG_OOUTGOING;
    }

    QString formatMsg = style.formatMessage(type, item.name, item.sendTime, item.messageText, formatFlag);

    itemWidget->setData(Qt::DisplayRole, qVariantFromValue(IMHistoryItemPainter(formatMsg)));
    itemWidget->setData(ROLE_HIID, item.hiid);
    itemWidget->setData(ROLE_OFFLINE, (type == ChatStyle::FORMATMSG_OOUTGOING) ? true : false);

    /* calculate plain text */
    QTextDocument doc;
    doc.setHtml(item.messageText);
    itemWidget->setData(ROLE_PLAINTEXT, doc.toPlainText());
}

QListWidgetItem *ImHistoryBrowser::createItem(IMHistoryItem &item)
{
    QListWidgetItem *itemWidget = new QListWidgetItem;
    fillItem(itemWidget, item);
    return itemWidget;
}

void ImHistoryBrowser::filterRegExpChanged()
{
    QString text = ui.filterPatternLineEdit->text();

    if (text.isEmpty()) {
        ui.clearFilterButton->hide();
    } else {
        ui.clearFilterButton->show();
    }

    filterItems();
}

void ImHistoryBrowser::clearFilter()
{
    ui.filterPatternLineEdit->clear();
    ui.filterPatternLineEdit->setFocus();
}

void ImHistoryBrowser::filterItems(QListWidgetItem *item)
{
    QString text = ui.filterPatternLineEdit->text();

    if (item == NULL) {
        int count = ui.listWidget->count();
        for (int i = 0; i < count; i++) {
            item = ui.listWidget->item(i);
            if (text.isEmpty()) {
                item->setHidden(false);
            } else {
                if (item->data(ROLE_PLAINTEXT).toString().contains(text, Qt::CaseInsensitive)) {
                    item->setHidden(false);
                } else {
                    item->setHidden(true);
                }
            }
        }
    } else {
        if (text.isEmpty()) {
            item->setHidden(false);
        } else {
            if (item->data(ROLE_PLAINTEXT).toString().contains(text, Qt::CaseInsensitive)) {
                item->setHidden(false);
            } else {
                item->setHidden(true);
            }
        }
    }
}

void ImHistoryBrowser::getSelectedItems(QList<int> &items)
{
    QList<QListWidgetItem*> itemWidgets = ui.listWidget->selectedItems();

    QList<QListWidgetItem*>::iterator it;
    for (it = itemWidgets.begin(); it != itemWidgets.end(); it++) {
        QListWidgetItem *item = *it;
        if (item->isHidden()) {
            continue;
        }
        items.append(item->data(ROLE_HIID).toString().toInt());
    }
}

void ImHistoryBrowser::itemSelectionChanged()
{
    QList<int> hiids;
    getSelectedItems(hiids);

    if (hiids.size()) {
        // activate buttons
        ui.copyButton->setEnabled(true);
        ui.removeButton->setEnabled(true);
    } else {
        // deactivate buttons
        ui.copyButton->setDisabled(true);
        ui.removeButton->setDisabled(true);
    }
}

void ImHistoryBrowser::customContextMenuRequested(QPoint pos)
{
    QList<int> hiids;
    getSelectedItems(hiids);

    QListWidgetItem *currentItem = ui.listWidget->currentItem();

    QMenu contextMnu(this);

    QAction *selectAll = new QAction(tr("Mark all"), &contextMnu);
    QAction *copyMessage = new QAction(tr("Copy"), &contextMnu);
    QAction *removeMessages = new QAction(tr("Delete"), &contextMnu);
    QAction *clearHistory = new QAction(tr("Clear history"), &contextMnu);

    QAction *sendItem = NULL;
    if (textEdit) {
        sendItem = new QAction(tr("Send"), &contextMnu);
        if (currentItem) {
            connect(sendItem, SIGNAL(triggered()), this, SLOT(sendMessage()));
        } else {
            sendItem->setDisabled(true);
        }
    }

    if (hiids.size()) {
        connect(selectAll, SIGNAL(triggered()), ui.listWidget, SLOT(selectAll()));
        connect(copyMessage, SIGNAL(triggered()), this, SLOT(copyMessage()));
        connect(removeMessages, SIGNAL(triggered()), this, SLOT(removeMessages()));
        connect(clearHistory, SIGNAL(triggered()), this, SLOT(clearHistory()));
    } else {
        selectAll->setDisabled(true);
        copyMessage->setDisabled(true);
        removeMessages->setDisabled(true);
        clearHistory->setDisabled(true);
    }

    contextMnu.addAction(selectAll);
    contextMnu.addSeparator();
    contextMnu.addAction(copyMessage);
    contextMnu.addAction(removeMessages);
    contextMnu.addAction(clearHistory);
    if (sendItem) {
        contextMnu.addSeparator();
        contextMnu.addAction(sendItem);
    }

    contextMnu.exec(QCursor::pos());
}

void ImHistoryBrowser::copyMessage()
{
    QListWidgetItem *currentItem = ui.listWidget->currentItem();
    if (currentItem) {
        int hiid = currentItem->data(ROLE_HIID).toString().toInt();
        IMHistoryItem item;
        if (historyKeeper.getMessage(hiid, item)) {
            QTextDocument doc;
            doc.setHtml(item.messageText);
            QApplication::clipboard()->setText(doc.toPlainText());
        }
    }
}

void ImHistoryBrowser::removeMessages()
{
    QList<int> hiids;
    getSelectedItems(hiids);

    historyKeeper.removeMessages(hiids);
}

void ImHistoryBrowser::clearHistory()
{
    historyKeeper.clear();
}

void ImHistoryBrowser::sendMessage()
{
    if (textEdit) {
        QListWidgetItem *currentItem = ui.listWidget->currentItem();
        if (currentItem) {
            int hiid = currentItem->data(ROLE_HIID).toString().toInt();
            IMHistoryItem item;
            if (historyKeeper.getMessage(hiid, item)) {
                textEdit->clear();
                textEdit->setText(item.messageText);
                textEdit->setFocus();
                QTextCursor cursor = textEdit->textCursor();
                cursor.movePosition(QTextCursor::End);
                textEdit->setTextCursor(cursor);
                close();
            }
        }
    }
}

void ImHistoryBrowser::privateChatChanged(int list, int type)
{
    if (m_isPrivateChat == false) {
        return;
    }

    if (list == NOTIFY_LIST_PRIVATE_OUTGOING_CHAT) {
        switch (type) {
        case NOTIFY_TYPE_ADD:
            {
                m_savedOfflineChat.clear();
                rsMsgs->getPrivateChatQueueCount(false) && rsMsgs->getPrivateChatQueue(false, m_peerId, m_savedOfflineChat);
            }
            break;
        case NOTIFY_TYPE_DEL:
            {
                m_savedOfflineChat.clear();
            }
            break;
        }

        // recalculate items in history
        int count = ui.listWidget->count();
        for (int i = 0; i < count; i++) {
            QListWidgetItem *itemWidget = ui.listWidget->item(i);

            if (itemWidget->data(ROLE_OFFLINE).toBool()) {
                int hiid = itemWidget->data(ROLE_HIID).toInt();

                IMHistoryItem item;
                if (historyKeeper.getMessage(hiid, item) == false) {
                    continue;
                }

                fillItem(itemWidget, item);
            }
        }

        filterRegExpChanged();
    }
}
