#include "records.h"
#include <set>

Records::Records(double s_range_)
  : s_range(s_range_), rec_size(80)
{

}

void Records::update(double time_stamp, const Telemetry &telemetry)
{
  // remove records of cars that are not in range anymore
  std::set<int> to_be_deleted;
  for (auto el(records.cbegin()); el!=records.cend(); ++el)
  {
    std::map<int,Detection>::const_iterator sense(telemetry.sensor_fusion.find(el->first));
    if (sense!=telemetry.sensor_fusion.cend())
    {
      if (abs(sense->second.s()-telemetry.car_s) > s_range)
        to_be_deleted.insert(el->first);
    }
    else
      to_be_deleted.insert(el->first);
  }
  for (auto el(to_be_deleted.cbegin()); el!=to_be_deleted.cend(); ++el)
  {
//    // debugging:
//    std::cout<<"-- "<<*el<<" > "<<records[*el]<<std::endl;
    records.erase(*el);
  }
  // extend records of cars in range; create new records if needed
  for (auto el(telemetry.sensor_fusion.cbegin()); el!=telemetry.sensor_fusion.cend(); ++el)
  {
    if (abs(el->second.s()-telemetry.car_s)<s_range)
    {
      if (records.find(el->first)==records.end())
      {
        records[el->first] = Record(el->first, rec_size);
//        // debugging:
//        std::cout<<"++ "<<el->first<<" [";
//        for (auto i(records.cbegin()); i!=records.cend(); ++i)
//        {
//          std::cout<<i->first<<" ";
//        }
//        std::cout<<"]"<<std::endl;
      }
      records[el->first].record(time_stamp, el->second);
    }
  }
}

