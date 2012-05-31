/* Copyright 2012 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __LIQUID_WATER_INTERNAL_PERSISTENT_MAP_H__
#define __LIQUID_WATER_INTERNAL_PERSISTENT_MAP_H__

#include <debug/cauv_debug.h>

#include <utility/string.h>

#include <QString>
#include <QMap>
#include <QPair>
#include <QSqlQuery>
#include <QSqlRecord>
#include <QSqlDatabase>
#include <QSqlError>
#include <QVariant>

namespace liquid{
namespace water{
namespace internal{

/* actually, it's a two-column database: if this turns out to be too slow, then
 * will do caching of the most recent values in memory.
 */

// This map always maps double:double
class Map{
    public:
        Map(QString name)
            : m_db(QSqlDatabase::addDatabase("QSQLITE")),
              m_table_name(name){
            const char* db_name = "/tmp/cauv/gui.db.sqlite";
            bool success;

            m_db.setDatabaseName(db_name);
            success = m_db.open();

            if(!success){
                error() << "could not open database:" << db_name;
                m_db.setDatabaseName(":memory:");
                success = m_db.open();
                if(!success){
                    throw std::runtime_error("SQLite3 support is required");
                }
            }

            // SQL ninjas are invited to optimise at will...

            QSqlQuery query(m_db);
            success = query.exec(QString(
                "CREATE TABLE IF NOT EXISTS %1 (k REAL PRIMARY KEY, v REAL)"
            ).arg(m_table_name));
            if(!success){
                throw std::runtime_error(QString("failed to create table '%1'").arg(name).toUtf8().data());
            }

            m_insert_query = QSqlQuery(m_db);
            m_insert_query.prepare(QString(
                "INSERT INTO %1 (k, v) VALUES (:k, :v)"
            ).arg(m_table_name));

            m_inrange_query = QSqlQuery(m_db);
            m_inrange_query.prepare(QString(
                "SELECT k, v FROM %1 WHERE k BETWEEN :low AND :high"
            ).arg(m_table_name));

            m_minavgmax_query = QSqlQuery(m_db);
            m_minavgmax_query.prepare(QString(
                "SELECT MIN(v), AVG(v), MAX(v) FROM %1 WHERE k BETWEEN :low AND :high"
            ).arg(m_table_name));

            m_closeto_query = QSqlQuery(m_db);
            m_closeto_query.prepare(QString(
                "SELECT k, v FROM %1 ORDER BY ABS(k - :value) LIMIT :n"
            ).arg(m_table_name));

            m_closest_query = QSqlQuery(m_db);
            m_closest_query.prepare(QString(
                "SELECT k, v FROM %1 ORDER BY ABS(k - :value) LIMIT 1"
            ).arg(m_table_name));
        }

        ~Map(){
            m_db.close();
        }


        void insert(double const& k, double const& v){
            m_insert_query.bindValue(":k", QVariant(k));
            m_insert_query.bindValue(":v", QVariant(v));

            bool success = m_insert_query.exec();
            if(!success)
                throw std::runtime_error(std::string(
                    mkStr() << "failed to exec insert query (duplicate?):"
                            << m_inrange_query.lastError().text().toUtf8().data()
                ));

            m_insert_query.finish();
        }

        // returns key: value
        QMap<double,double> valuesInRange(double const& low, double const& high){
            m_inrange_query.bindValue(":low", QVariant(low));
            m_inrange_query.bindValue(":high", QVariant(high));
            bool success = m_inrange_query.exec();
            if(!success)
                throw std::runtime_error(std::string(
                    mkStr() << "failed to exec range query:"
                            << m_inrange_query.lastError().text().toUtf8().data()
                ));

            //debug() << "query:" << m_inrange_query.lastQuery().toUtf8().data();

            QMap<double, double> r;
            while(m_inrange_query.next()){
                r[m_inrange_query.value(0).toDouble()] = m_inrange_query.value(1).toDouble();
            }
            m_inrange_query.finish();

            return r;
        }

        struct MinAvgMax{
            double min; double average; double max;
        };

        MinAvgMax minAvgMaxOfRange(double const& low, double const& high){
            m_minavgmax_query.bindValue(":low", QVariant(low));
            m_minavgmax_query.bindValue(":high", QVariant(high));
            bool success = m_minavgmax_query.exec();
            if(!success)
                throw std::runtime_error(std::string(
                    mkStr() << "failed to exec min/avg/max query:"
                            << m_minavgmax_query.lastError().text().toUtf8().data()
                ));

            success = m_minavgmax_query.first();
            if(!success)
                throw std::runtime_error(std::string(
                    mkStr() << "failed to exec min/avg/max query (no data):"
                            << m_minavgmax_query.lastError().text().toUtf8().data()
                ));

            //debug() << "query:" << m_minavgmax_query.lastQuery().toUtf8().data();

            MinAvgMax r = {
                m_minavgmax_query.value(0).toDouble(),
                m_minavgmax_query.value(1).toDouble(),
                m_minavgmax_query.value(2).toDouble()
            };
            m_minavgmax_query.finish();
            return r;
        }

        // returns key: value
        QMap<double,double> valuesCloseTo(double const& k, uint32_t n){
            m_closeto_query.bindValue(":n", QVariant(n));
            m_closeto_query.bindValue(":value", QVariant(k));
            bool success = m_closeto_query.exec();
            if(!success)
                throw std::runtime_error(std::string(
                    mkStr() << "failed to exec close-to query:"
                            << m_closeto_query.lastError().text().toUtf8().data()
                ));

            //debug() << "query:" << m_closeto_query.lastQuery().toUtf8().data();

            QMap<double, double> r;
            while(m_closeto_query.next()){
                r[m_closeto_query.value(0).toDouble()] = m_closeto_query.value(1).toDouble();
            }
            m_inrange_query.finish();

            return r;
        }

        // returns key, value
        QPair<double,double> valueClosestTo(double const& k){
            m_closest_query.bindValue(":value", QVariant(k));
            bool success = m_closest_query.exec();
            if(!success)
                throw std::runtime_error(std::string(
                    mkStr() << "failed to exec closest-to query:"
                            << m_closest_query.lastError().text().toUtf8().data()
                ));

            success = m_closest_query.first();
            if(!success)
                throw std::runtime_error(std::string(
                    mkStr() << "failed to exec closest-to query (no closest item):"
                            << m_closest_query.lastError().text().toUtf8().data()
                ));

            //debug() << "query:" << m_closest_query.lastQuery().toUtf8().data();

            QPair<double, double> r(
                m_closest_query.value(0).toDouble(),
                m_closest_query.value(1).toDouble()
            );
            m_closest_query.finish();

            return r;
        }

    private:
        QSqlDatabase m_db;
        QString m_table_name;

        QSqlQuery m_insert_query;
        QSqlQuery m_inrange_query;
        QSqlQuery m_minavgmax_query;
        QSqlQuery m_closeto_query;
        QSqlQuery m_closest_query;
};

} // internal
} // water
} // liquid


#endif // ndef __LIQUID_WATER_INTERNAL_PERSISTENT_MAP_H__
