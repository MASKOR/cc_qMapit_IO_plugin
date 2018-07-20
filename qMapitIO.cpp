//##########################################################################
//#                                                                        #
//#         CLOUDCOMPARE PLUGIN: qMapitIO                                  #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: CloudCompare project                               #
//#                     Tobias Neumann                                     #
//#                                                                        #
//##########################################################################

#include "qMapitIO.h"

#include "IcmFilter.h"

qMapitIO::qMapitIO( QObject* parent ) :
	QObject( parent )
  , ccIOFilterPluginInterface( ":/CC/plugin/qMapitIO/info.json" )
{
}

void qMapitIO::registerCommands( ccCommandLineInterface *cmd )
{ }

QVector<FileIOFilter::Shared> qMapitIO::getFilters()
{
	return QVector<FileIOFilter::Shared>{
		FileIOFilter::Shared( new MapitFilter ),
	};
}

