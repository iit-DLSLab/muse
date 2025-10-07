#ifndef IIT_COMMONS_FILTERS_H_
#define IIT_COMMONS_FILTERS_H_

namespace iit {
namespace commons {


template<typename VT >
struct Filter
{
    typedef VT Value_t;
    virtual ~Filter() {}
    virtual Value_t filter(const Value_t&) = 0;
    virtual void    reset (const Value_t&) = 0;

};


/**
 * Simple first order alpha filter
 */
template<typename T>
struct AlphaFilter : public Filter<T>
{
    typedef T Value_t;

    AlphaFilter(double a, const Value_t& start) : alpha(a), state1(start) {}

    Value_t filter(const Value_t& in) {
        return state1 = (1-alpha)*state1 + alpha*in;
    }

    void reset(const Value_t& val) {
        state1 = val;
    }
private:
    double alpha;
    Value_t state1;
};

template<typename VT>
struct FilterPipe : public Filter< VT >
{
    //TODO compile time constraint that F1 and F2 have the same value-type
    typedef VT Value_t;
    typedef Filter<Value_t> Filter_t;

    FilterPipe(Filter_t& fi1, Filter_t& fi2) : f1(&fi1), f2(&fi2) {}

    Value_t filter(const Value_t& in) {
        return f2->filter( f1->filter(in) );
    }

    void reset(const Value_t& val) {
        f1->reset(val);
        f2->reset(val);
    }
private:
    Filter_t* f1;
    Filter_t* f2;
};

/**
 * Constructs a filter-pipe
 */
template<typename VT> inline
FilterPipe<VT> make_filtpipe(Filter<VT>& fi1, Filter<VT>& fi2) {
    return FilterPipe<VT>(fi1, fi2);
}

/**
 * Simple second order filter
 */
template<typename T>
struct SecondOrderAlphaFilter : public Filter<T>
{
    SecondOrderAlphaFilter(double a, const T& start) : alpha(a), state1(start), state2(start) {}

    T filter(const T& in) {
        state1 = (1-alpha)*state1 + alpha*in;
        state2 = (1-alpha)*state2 + alpha*state1;
        return state2;
    }

    void reset(const T& val) {
        state1 = state2 = val;
    }
private:
    double alpha;
    T state1;
    T state2;
};

template<typename Value_t>
struct Limits {
    Value_t min;
    Value_t max;
    Limits(const Value_t& m1, const Value_t& m2) : min(m1), max(m2) {}
};

template<typename T> inline
Limits<T> make_limits(const T& m1, const T& m2) {
    return Limits<T>(m1,m2);
}




template<typename VT>
struct FilteredVar
{
    typedef VT Value_t;
    typedef Filter<Value_t> Filter_t;

    FilteredVar(const Value_t& v, const Limits<Value_t>& lim, Filter_t& filt)
        : val(v), limits(lim), filter(&filt) {}

    void changeFilter(Filter_t& filt) { filter = &filt; }

    const Value_t& value() const { return val; }

    const Value_t& update(const Value_t& demand)
    {
        val = filter->filter(demand);
        cap(val);
        return value();
    }

    void hardSet(const Value_t& v)
    {
        val = v;
        cap(val);
        filter->reset(val);
    }
private:
    void cap(Value_t& v) const
    {
        v = v>limits.max ? limits.max : (v<limits.min? limits.min : v);
    }

    Value_t val;
    Limits<Value_t> limits;
    Filter_t* filter;
};

template<typename VT> inline
FilteredVar<VT> make_fvar(
        const VT& v,
        const Limits<VT>& lim,
        Filter<VT>& filt)
{
    return FilteredVar<VT>(v, lim, filt);
}

}
}

#endif
