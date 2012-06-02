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

#include <stdexcept>

#include <QString>
#include <QMap>
#include <QList>
#include <QVariant>
#include <QPair>
#include <QSqlQuery>
#include <QSqlDatabase>
#include <QSqlError>

#include <utility/string.h>

namespace liquid{
namespace water{
namespace internal{

/* actually, it's a two-column database: if this turns out to be too slow, then
 * will do caching of the most recent values in memory.
 *
 * MUST ONLY BE ACCESSED FROM ONE THREAD
 *
 */

// This map always maps double:double
class Map{
    public:
        Map(QString name);
        ~Map(); 


        struct MinAvgMax{
            double min; double average; double max;
        };
        
        /* insert
         *  k: key
         *  v: value
         */
        void insert(double const& k, double const& v);
        
        // iteratorT must have ->first and ->second for the key and value
        // respectively (an iterator to a vector of std::pair will do)
        template<typename iteratorT>
        void insertMultiple(iteratorT begin, iteratorT end){
            QVariantList k_list;
            QVariantList v_list;
            while(begin != end){
                k_list.append(QVariant(double(begin->first)));
                v_list.append(QVariant(double(begin->second)));
                begin++;
            }
            m_insert_query.bindValue(":k", k_list);
            m_insert_query.bindValue(":v", v_list);

            m_db.transaction();
            bool success = m_insert_query.execBatch();
            if(!success)
                throw std::runtime_error(std::string(
                    mkStr() << "failed to exec insert query (duplicate?):"
                            << m_inrange_query.lastError().text().toUtf8().data()
                ));

            m_insert_query.finish();
            m_db.commit();
        }

        // returns key: value
        QMap<double,double> valuesInRange(double const& low, double const& high);
        
        // calculates MinAvgMax{min value, average value, max value} from range
        // of keys, return value is the number of contributing values (check
        // for zero!)
        uint32_t minAvgMaxOfRange(double const& low, double const& high, MinAvgMax& output);

        // returns key: value
        QMap<double,double> valuesCloseTo(double const& k, uint32_t n);

        // returns key, value
        QPair<double,double> valueClosestTo(double const& k);

    private:
        QString m_table_name;

        QSqlDatabase m_db;
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
