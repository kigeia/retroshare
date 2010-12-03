/****************************************************************
 * This file is distributed under the following license:
 *
 * Copyright (c) 2008, defnax
 * Copyright (C) 2006  Christophe Dumez
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 *************************************************************************/

#include <QString>
#include <QDir>
#include <QFileDialog>

#include "misc.h"

// return best userfriendly storage unit (B, KiB, MiB, GiB, TiB)
// use Binary prefix standards from IEC 60027-2
// see http://en.wikipedia.org/wiki/Kilobyte
// value must be given in bytes
QString misc::friendlyUnit(float val)
{
    if(val < 0) {
        return tr("Unknown", "Unknown (size)");
    }
    const QString units[4] = {tr(" B", "bytes"), tr(" KiB", "kibibytes (1024 bytes)"), tr(" MiB", "mebibytes (1024 kibibytes)"), tr(" GiB", "gibibytes (1024 mibibytes)")};
    for(unsigned int i=0; i<5; ++i) {
        if (val < 1024.) {
            return QString(QByteArray::number(val, 'f', 1)) + units[i];
        }
        val /= 1024.;
    }
    return  QString(QByteArray::number(val, 'f', 1)) + tr(" TiB", "tebibytes (1024 gibibytes)");
}

bool misc::isPreviewable(QString extension)
{
    extension = extension.toUpper();
    if(extension == "AVI") return true;
    if(extension == "MP3") return true;
    if(extension == "OGG") return true;
    if(extension == "OGM") return true;
    if(extension == "WMV") return true;
    if(extension == "WMA") return true;
    if(extension == "MPEG") return true;
    if(extension == "MPG") return true;
    if(extension == "ASF") return true;
    if(extension == "QT") return true;
    if(extension == "RM") return true;
    if(extension == "RMVB") return true;
    if(extension == "RMV") return true;
    if(extension == "SWF") return true;
    if(extension == "FLV") return true;
    if(extension == "WAV") return true;
    if(extension == "MOV") return true;
    if(extension == "VOB") return true;
    if(extension == "MID") return true;
    if(extension == "AC3") return true;
    if(extension == "MP4") return true;
    if(extension == "MP2") return true;
    if(extension == "AVI") return true;
    if(extension == "FLAC") return true;
    if(extension == "AU") return true;
    if(extension == "MPE") return true;
    if(extension == "MOV") return true;
    if(extension == "MKV") return true;
    if(extension == "AIF") return true;
    if(extension == "AIFF") return true;
    if(extension == "AIFC") return true;
    if(extension == "RA") return true;
    if(extension == "RAM") return true;
    if(extension == "M4P") return true;
    if(extension == "M4A") return true;
    if(extension == "3GP") return true;
    if(extension == "AAC") return true;
    if(extension == "SWA") return true;
    if(extension == "MPC") return true;
    if(extension == "MPP") return true;
    return false;
}

// return qBittorrent config path
QString misc::qBittorrentPath()
{
    QString qBtPath = QDir::homePath()+QDir::separator()+QString::fromUtf8(".qbittorrent") + QDir::separator();
    // Create dir if it does not exist
    if(!QFile::exists(qBtPath)){
        QDir dir(qBtPath);
        dir.mkpath(qBtPath);
    }
    return qBtPath;
}

QString misc::findFileInDir(QString dir_path, QString fileName)
{
    QDir dir(dir_path);
    if(dir.exists(fileName)) {
        return dir.filePath(fileName);
    }
    QStringList subDirs = dir.entryList(QDir::Dirs);
    QString subdir_name;
    foreach(subdir_name, subDirs) {
        QString result = findFileInDir(dir.path()+QDir::separator()+subdir_name, fileName);
        if(!result.isNull()) {
            return result;
        }
    }
    return QString();
}

// Can't use template class for QString because >,< use unicode code for sorting
// which is not what a human would expect when sorting strings.
void misc::insertSortString(QList<QPair<int, QString> > &list, QPair<int, QString> value, Qt::SortOrder sortOrder)
{
    int i = 0;
    if(sortOrder == Qt::AscendingOrder) {
        while(i < list.size() and QString::localeAwareCompare(value.second, list.at(i).second) > 0) {
            ++i;
        }
    }else{
        while(i < list.size() and QString::localeAwareCompare(value.second, list.at(i).second) < 0) {
            ++i;
        }
    }
    list.insert(i, value);
}

float misc::getPluginVersion(QString filePath)
{
    QFile plugin(filePath);
    if(!plugin.exists()){
        qDebug("%s plugin does not exist, returning 0.0", filePath.toUtf8().data());
        return 0.0;
    }
    if(!plugin.open(QIODevice::ReadOnly | QIODevice::Text)){
        return 0.0;
    }
    float version = 0.0;
    while (!plugin.atEnd()){
        QByteArray line = plugin.readLine();
        if(line.startsWith("#VERSION: ")){
            line = line.split(' ').last();
            line.replace("\n", "");
            version = line.toFloat();
            qDebug("plugin %s version: %.2f", filePath.toUtf8().data(), version);
            break;
        }
    }
    return version;
}

// Take a number of seconds and return an user-friendly
// time duration like "1d 2h 10m".
QString misc::userFriendlyDuration(qlonglong seconds)
{
    if(seconds < 0) {
        return tr("Unknown");
    }
    if(seconds < 60) {
        return tr("< 1m", "< 1 minute");
    }
    int minutes = seconds / 60;
    if(minutes < 60) {
        return tr("%1 minutes","e.g: 10minutes").arg(QString::QString::fromUtf8(misc::toString(minutes).c_str()));
    }
    int hours = minutes / 60;
    minutes = minutes - hours*60;
    if(hours < 24) {
        return tr("%1h %2m", "e.g: 3hours 5minutes").arg(QString::fromUtf8(misc::toString(hours).c_str())).arg(QString::fromUtf8(misc::toString(minutes).c_str()));
    }
    int days = hours / 24;
    hours = hours - days * 24;
    if(days < 365) {
        return tr("%1d %2h", "e.g: 2days 10hours").arg(QString::fromUtf8(misc::toString(days).c_str())).arg(QString::fromUtf8(misc::toString(hours).c_str()));
    }
    int years = days / 365;
    days = days - years * 365;
    return tr("%1y %2d", "e.g: 2 years 2days ").arg(QString::fromUtf8(misc::toString(years).c_str())).arg(QString::fromUtf8(misc::toString(days).c_str()));
}

QString misc::userFriendlyUnit(double count, unsigned int decimal, double factor)
{
    if (count <= 0.0) {
        return "0";
    }

    QString output;

    int i;
    for (i = 0; i < 5; i++) {
        if (count < factor) {
            break;
        }

        count /= factor;
    }

    QString unit;
    switch (i) {
    case 0:
        decimal = 0; // no decimal
        break;
    case 1:
        unit = tr("k", "e.g: 3.1 k");
        break;
    case 2:
        unit = tr("M", "e.g: 3.1 M");
        break;
    case 3:
        unit = tr("G", "e.g: 3.1 G");
        break;
    default: // >= 4
        unit = tr("T", "e.g: 3.1 T");
    }

    return QString("%1 %2").arg(count, 0, 'f', decimal).arg(unit);
}

QString misc::removeNewLine(const QString &text)
{
    return QString(text).replace("\n", " ");
}

QString misc::removeNewLine(const std::string &text)
{
    return QString::fromUtf8(text.c_str()).replace("\n", " ");
}

QString misc::removeNewLine(const std::wstring &text)
{
    return QString::fromStdWString(text).replace("\n", " ");
}

bool misc::getOpenFileName(QWidget *parent, RshareSettings::enumLastDir type, const QString &caption, const QString &filter, QString &file)
{
    QString lastDir = Settings->getLastDir(type);

    file = QFileDialog::getOpenFileName(parent, caption, lastDir, filter, NULL, QFileDialog::DontResolveSymlinks);

    if (file.isEmpty() == false) {
        lastDir = QFileInfo(file).absoluteDir().absolutePath();
        Settings->setLastDir(type, lastDir);
    }

    return !file.isEmpty();
}

bool misc::getOpenFileNames(QWidget *parent, RshareSettings::enumLastDir type, const QString &caption, const QString &filter, QStringList &files)
{
    QString lastDir = Settings->getLastDir(type);

    files = QFileDialog::getOpenFileNames(parent, caption, lastDir, filter, NULL, QFileDialog::DontResolveSymlinks);

    if (files.isEmpty() == false) {
        lastDir = QFileInfo(*files.begin()).absoluteDir().absolutePath();
        Settings->setLastDir(type, lastDir);
    }

    return !files.isEmpty();
}

bool misc::getSaveFileName(QWidget *parent, RshareSettings::enumLastDir type, const QString &caption, const QString &filter, QString &file)
{
    QString lastDir = Settings->getLastDir(type);

    file = QFileDialog::getSaveFileName(parent, caption, lastDir, filter);

    if (file.isEmpty() == false) {
        lastDir = QFileInfo(file).absoluteDir().absolutePath();
        Settings->setLastDir(type, lastDir);
    }

    return !file.isEmpty();
}
