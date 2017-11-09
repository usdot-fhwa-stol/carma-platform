#ifndef H_CAN_DISPATCHER
#define H_CAN_DISPATCHER

#include <socketcan_interface/interface.h>
#include <list>
#include <boost/thread/mutex.hpp>
#include <boost/unordered_map.hpp>
#include <boost/utility.hpp>
#include <boost/foreach.hpp>
#include <boost/weak_ptr.hpp>

namespace can{

template< typename Listener > class SimpleDispatcher{
public:
    typedef typename Listener::Callable Callable;
    typedef typename Listener::Type Type;
protected:
    class DispatcherBase : boost::noncopyable{
        class GuardedListener: public Listener{
            boost::weak_ptr<DispatcherBase> guard_;
        public:
            GuardedListener(boost::shared_ptr<DispatcherBase> g, const Callable &callable): Listener(callable), guard_(g){}
            virtual ~GuardedListener() {
                boost::shared_ptr<DispatcherBase> d = guard_.lock();
                if(d){
                    d->remove(this);
                }
            }
        };
        
        boost::mutex &mutex_;
        std::list< Listener* > listeners_;
    public:
        DispatcherBase(boost::mutex &mutex) : mutex_(mutex) {}
        void dispatch_nolock(const Type &obj) const{
           for(typename std::list<Listener* >::const_iterator it=listeners_.begin(); it != listeners_.end(); ++it){
               (**it)(obj);
            }
        }
        void remove(Listener *d){
            boost::mutex::scoped_lock lock(mutex_);
            listeners_.remove(d);
        }
        size_t numListeners(){
            boost::mutex::scoped_lock lock(mutex_);
            return listeners_.size();
        }

        static typename Listener::Ptr createListener(boost::shared_ptr<DispatcherBase> dispatcher, const  Callable &callable){
            boost::shared_ptr<Listener > l(new GuardedListener(dispatcher,callable));
            dispatcher->listeners_.push_back(l.get());
            return l;
        }
    };
    boost::mutex mutex_;
    boost::shared_ptr<DispatcherBase> dispatcher_;
public:
    SimpleDispatcher() : dispatcher_(new DispatcherBase(mutex_)) {}
    typename Listener::Ptr createListener(const Callable &callable){
        boost::mutex::scoped_lock lock(mutex_);
        return DispatcherBase::createListener(dispatcher_, callable);
    }
    void dispatch(const Type &obj){
        boost::mutex::scoped_lock lock(mutex_);
        dispatcher_->dispatch_nolock(obj);
    }
    size_t numListeners(){
        return dispatcher_->numListeners();
    }
    operator Callable() { return Callable(this,&SimpleDispatcher::dispatch); }
};

template<typename K, typename Listener, typename Hash = boost::hash<K> > class FilteredDispatcher: public SimpleDispatcher<Listener>{
    typedef SimpleDispatcher<Listener> BaseClass;
    boost::unordered_map<K, boost::shared_ptr<typename BaseClass::DispatcherBase >, Hash> filtered_;
public:
    using BaseClass::createListener;
    typename Listener::Ptr createListener(const K &key, const typename BaseClass::Callable &callable){
        boost::mutex::scoped_lock lock(BaseClass::mutex_);
        boost::shared_ptr<typename BaseClass::DispatcherBase > &ptr = filtered_[key];
        if(!ptr) ptr.reset(new typename BaseClass::DispatcherBase(BaseClass::mutex_));
        return BaseClass::DispatcherBase::createListener(ptr, callable);
    }
    void dispatch(const typename BaseClass::Type &obj){
        boost::mutex::scoped_lock lock(BaseClass::mutex_);
        boost::shared_ptr<typename BaseClass::DispatcherBase > &ptr = filtered_[obj];
        if(ptr) ptr->dispatch_nolock(obj);
        BaseClass::dispatcher_->dispatch_nolock(obj);
    }
    operator typename BaseClass::Callable() { return typename BaseClass::Callable(this,&FilteredDispatcher::dispatch); }
};

} // namespace can
#endif
