#include "detection.h"

Detection::Detection()
  : id(-1), v(6)
{ }

Detection::Detection(const json &detection)
  : id(detection[0]), v(6)
{
  for (int i(1); i<detection.size(); ++i)
  {
    v[i-1]=detection[i];
  }
}

Detection::Detection(const Detection & detection)
  : id(detection.id), v(detection.v)
{ }

Detection::Detection(int id_, const dvector &v_)
  : id(id_), v(v_)
{ }

double Detection::x() const
{
  return v[0];
}

double Detection::y() const
{
  return v[1];
}

dvector Detection::pos() const
{
  return {x(), y()};
}

double Detection::vx() const
{
  return v[2];
}

double Detection::vy() const
{
  return v[3];
}

dvector Detection::speed() const
{
  return {vx(), vy()};
}

double Detection::s() const
{
  return v[4];
}

double Detection::d() const
{
  return v[5];
}

std::ostream & operator<<(std::ostream &out, const Detection & d)
{
  out<<"id: "<<d.id<<", x: "<<d.x()<<", y: "<<d.y()<<", vx: "<<d.vx()<<", vy: "<<d.vy()<<", s: "<<d.s()<<", d: "<<d.d();
  return out;
}


Record::Record()
  : id(-1), v(0), current(0), full(false)
{ }

Record::Record(int id_, int size_=20)
  : id(id_), v(size_), current(0), full(false)
{ }

Record::Record(const Record &rec)
  : id(rec.id), v(rec.v), current(rec.current), full(rec.full)
{ }

void Record::record(double time_stamp, const Detection &d)
{
  v[current] = std::pair<double, dvector>(time_stamp, d.v);
  current++; current%=v.size();
  if (current==0)
    full=true;
}

std::pair<double, Detection> Record::operator[](int i) const
{
  int read_posn((i + current) % size());
  if (!is_full() & read_posn>=current)
  {
    std::stringstream ss;
    ss<<"cannot read from position "<<i<<" since the record ";
    if (current==0)
      ss<<"is empty.";
    else
      ss<<"has only "<<current<<"elements.";
  }
  else
  {
    const std::pair<double, dvector> & element(v[read_posn]);
    return std::pair<double, Detection>(element.first, Detection(id, element.second));
  }
}

std::pair<double, Detection> Record::get_last() const
{
  return operator[](size()-1);
}

int Record::size() const
{
  return v.size();
}

int Record::count() const
{
  return (is_full() ? size() : current);
}

bool Record::is_full() const
{
  return full;
}

std::ostream & operator<<(std::ostream &out, const Record &r)
{
  for (int i(r.size()-1); i>=r.size()-r.count(); --i)
  {
    out<<" ["<<r[i].first<<" : "<<r[i].second<<"]";
  }

//  for (auto d(r.v.cbegin()); d!=r.v.cend(); ++d)
//  {
//    out<<"["<<d->first<<" : ";
//    for (auto i(d->second.cbegin()); i!=d->second.cend(); ++i)
//    {
//      out<<*i<<" ";
//    }
//    out<<"] ";
//  }

  return out;
}
