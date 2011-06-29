#ifndef STREAMS_H
#define STREAMS_H

#include <boost/make_shared.hpp>
#include <boost/unordered_map.hpp>

#include <generated/messages.h>

#include <QObject>
#include <sstream>

namespace cauv {
    namespace gui {


        class DataStreamBase {

        public:

            template <class T>
                class AbstractChangeHandler {
                public:
                virtual void update(T value) = 0;
                virtual void set(T value) = 0;
                virtual bool isMutable() = 0;
            };

            template<class T, class streamtype>
            class DefaultChangeHandler : public AbstractChangeHandler<T> {
            public:
                DefaultChangeHandler(streamtype * stream) : m_stream(stream) {
                }

                virtual void update(T value) {
                    m_stream->notifyUpdate(value);
                }

                virtual void set(T){
                    // do nothing by default
                }

                virtual bool isMutable(){
                    return false;
                }

            protected:
                streamtype * m_stream;
            };


            DataStreamBase(const std::string name, const std::string units, DataStreamBase* parent = NULL):
                    m_parent(parent), m_name(name), m_units(units) {
            }

            virtual ~DataStreamBase(){}

            virtual const std::string getName(bool full=true) const {
                std::stringstream stream;

                if (full && m_parent != NULL)
                    stream << m_parent->getName() << " " << m_name;
                else stream << m_name;

                return stream.str();
            }

            virtual const std::string getUnits() const {
                return m_units;
            }

        protected:

            const DataStreamBase* m_parent;
            const std::string m_name;
            const std::string m_units;
        };


        template<class T>
        class AbstractDataStream  : public QObject, public DataStreamBase {
        public:
            typedef T type;

            AbstractDataStream(boost::shared_ptr<AbstractChangeHandler<T> > handler, std::string name, std::string units="", DataStreamBase *parent=NULL):
                DataStreamBase(name, units, parent),
                m_change(handler)
            {
            }

            void update(T value){
                m_change->update(value);
                m_latest = value;
            }

            void set(T value) {
                m_change->set(value);
                update(value);
            }

            T latest(){
                return m_latest;
            }

            bool isMutable() {
                return m_change->isMutable();
            }

            virtual void notifyUpdate(T value) = 0;

            void setChangeHandler(boost::shared_ptr<AbstractChangeHandler<T> > change){
                m_change = change;
            }

            protected:
            boost::shared_ptr<AbstractChangeHandler<T> > m_change;
            T m_latest;

        };



        class FloatStream : public AbstractDataStream<float> {
            Q_OBJECT

        public:
            FloatStream(std::string name, std::string units="", DataStreamBase *parent=NULL) :
                    AbstractDataStream<float>(boost::make_shared<DataStreamBase::DefaultChangeHandler<float, FloatStream> >(this), name, units, parent){
            }

            void notifyUpdate(float f) {
                Q_EMIT updated(f);
            }

        Q_SIGNALS:
            void updated(float f);
        };



        class IntStream : public AbstractDataStream<int> {
            Q_OBJECT

        public:

            IntStream(std::string name, std::string units="", DataStreamBase *parent=NULL) :
                    AbstractDataStream<int>(boost::make_shared<DefaultChangeHandler<int, IntStream> >(this), name, units, parent){
            }

            void notifyUpdate(int i) {
                Q_EMIT updated(i);
            }

        Q_SIGNALS:
            void updated(int i);
        };

    } // namespace gui
} // namespace cauv


#endif // STREAMS_H
