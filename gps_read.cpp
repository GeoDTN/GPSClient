/*
 * gps_read.cpp
 *
 * Copyright (C) 2015 TNO
 *
 * Written-by: Tadewos Somano <Tadewos.SomanoEwalo@tno.nl,tadewos85@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include"gps_read.h"
#include <stdlib.h>
#include <core/EventDispatcher.h>
#include<Configuration.h>
         namespace dtn
             {
                namespace GPSClient
                      {
                       const  int equatorial_radius=6378200;
                        const int polar_radius =6356750;
                         const double pi=3.14159265359;
                         const std::string gps_read::TAG = "gps_read";

                        	bool Configuration::Gpsd::enabled() const
                        		{
                        			return _enabled;
                        		}

                        		string Configuration::Gpsd::getHost() const
                        		{
                        			return _host;
                        		}

                        		unsigned int Configuration::Gpsd::getPort() const
                        		{
                        			return _port;
                        		}

                          		bool Configuration::P2P::enabled() const
                          		{
                          			return _enabled;

                      }
                          		Position::Position(): _lat(0), _lon(0), _alt(0)
                          		              {
                          		                }

                          				Position::Position(double lat, double lon, double alt)
                          				{
                          		                  _lat = lat;
                          		                  _lon = lon;
                          		                  _alt = alt;
                          				}

                          				Position::~Position()
                          				{
                          				}

                          				std::string Position::toString() const
                          				{

                          		                  stringstream stream;
                          		                  stream << "[lat=" << _lat << "[,lon=" << _lon << ",alt=" << _alt << "speed"<<_speed<<"climb"<<_climb<<"heading"<<_heading<<"time"<<_time<<"]";
                          		                  return stream.str ();
                          				}

                          				double Position::getLatitude () const
                          				{
                          					return _lat;
                          				}

                          				double Position::getLongitude () const
                          				{
                          					return _lon;
                          				}

                          				double Position::getAltitude () const
                          				{
                          					return _alt;
                          				}

                          		                void Position::setPosition (double lat, double lon, double alt,double speed,double climb,float heading,long int time)
                          		                {
                          		                  _lat = lat;
                          		                  _lon = lon;
                          		                  _alt = alt;
                          		                  _speed=speed;
                          		                  _climb=climb;
                          		                  _heading=heading;
                          		                  _time=time;
                          		                }




                          		              		Position::Position(): _lat(0), _lon(0), _alt(0),_speed(0),_climb(0),_heading(0),_time(0)
                          		                              {
                          		                              }

                          		              		Position::Position(double lat, double lon, double alt,double speed,double climb,float heading,long int time)
                          		              		{
                          		                                _lat = lat;
                          		                                _lon = lon;
                          		                                _alt = alt;
                          		                                _speed=speed;
                          		                                _climb=climb;
                          		                                _heading=heading;
                          		                                _time=time;
                          		              		}

                          		              		Position::~Position()
                          		              		{
                          		              		}

                          		              		std::string Position::toString() const
                          		              		{

                          		                                stringstream stream;
                          		                                stream << "[lat=" << _lat << "[,lon=" << _lon << ",alt=" << _alt << "speed"<<_speed <<"climb"<<_climb<<"heading"<<_heading<<"_time"<<time<< "]";
                          		                                return stream.str ();
                          		              		}

                          		              		double Position::getLatitude () const
                          		              		{
                          		              			return _lat;
                          		              		}

                          		              		double Position::getLongitude () const
                          		              		{
                          		              			return _lon;
                          		              		}

                          		              		double Position::getAltitude () const
                          		              		{
                          		              			return _alt;
                          		              		}
                          		              	double Position::getSpeed () const
                          		              	                          		              		{
                          		              	                          		              			return _speed;
                          		              	                          		              		}
                          		              double Position::getClimb () const
                          		                                        		              		{
                          		                                        		              			return _climb;
                          		                                        		              		}
                          		            double Position::getHeading () const
                          		                                      		              		{
                          		                                      		              			return _heading;
                          		                                      		              		}
                          		          double Position::getTime () const
                          		                                    		              		{
                          		                                    		              			return _time;
                          		                                    		              		}
                          		                              void Position::setPosition (double lat, double lon, double alt,double speed,double climb,double heading,long int time)
                          		                              {
                          		                                _lat = lat;
                          		                                _lon = lon;
                          		                                _alt = alt;
                          		                                _speed=speed;
                          		                              _climb=climb;
                          		                              _heading=heading;
                          		                              _time=time;

                          		                              }



                          		                            PositionEvent::PositionEvent(const dtn::GPSClient::Position &position, const PositionEventAction action)
                          		                           	: m_position(position), m_action(action)
                          		                            		{

                          		                            		}

                          		                            		PositionEvent::~PositionEvent()
                          		                            		{

                          		                            		}

                          		                            		PositionEventAction PositionEvent::getAction() const
                          		                            		{
                          		                            			return m_action;
                          		                            		}

                          		                            		const dtn::data::Position& PositionEvent::getPosition() const
                          		                            		{
                          		                            			return m_position;
                          		                            		}

                          		                            		const std::string PositionEvent::getName() const
                          		                            		{
                          		                            			return "PositionEvent";
                          		                            		}

                          		                            		void PositionEvent::raise(const dtn::data::Position &position, const PositionEventAction action)
                          		                            		{
                          		                           			// raise the new event; object will be deleted by event dispatcher.
                          		                           			dtn::core::EventDispatcher<PositionEvent>::raise( new PositionEvent(position, action) );
                          		                           		}

                          		                           		std::string PositionEvent::getMessage() const
                          		                            		{
                          		                            			if (getAction() == POSITION_FIX) {
                          		                            				return "new position is " + getPosition().toString();
                          		                            			}
                          		                            			return "unknown";
                          		                            		}

                          		                           	void GpsdClient::componentDown() throw () {
                          		                           	    ibrcommon::MutexLock l(_gpsd_connection_mutex);
                          		                           	      _shutdown = true;
                          		                           	      if (_gpsd_connected) {
                          		                           	        (void) gps_stream(&_gps_data, WATCH_DISABLE, NULL);
                          		                           	        gps_close (&_gps_data);
                          		                           	        _gpsd_connected = false;
                          		                           	         IBRCOMMON_LOGGER_TAG(GpsdClient::TAG, info) << "Closed down GpsdClient" << IBRCOMMON_LOGGER_ENDL;
                          		                           	      }
                          		                           	    }

                          		                           	    void GpsdClient::__cancellation() throw () {

                          		                           	    }

                          		                           	    void GpsdClient::onConfigurationChanged(const dtn::daemon::Configuration &config) throw () {
                          		                           	      reconnectGpsd(config);
                          		                           	    }



                          		                           	GpsdClient::GpsdClient()
                          		                           				    : _shutdown(false)
                          		                           				    {
                          		                           				    }

                          		                           				    GpsdClient::~GpsdClient() {
                          		                           				    }

                          		                           				    const std::string GpsdClient::TAG = "GpsdClient";

                          		                           				    const std::string GpsdClient::getName() const {
                          		                           				      return "GpsdClient";
                          		                           				    }

                          		                           				    void GpsdClient::reconnectGpsd(const dtn::daemon::Configuration &config) throw () {
                          		                           				      ibrcommon::MutexLock l(_gpsd_connection_mutex);
                          		                           				      const char *server = config.getGpsd().getHost().c_str();
                          		                           				      char portBuffer[12];
                          		                           				      snprintf (portBuffer, 12, "%u", config.getGpsd().getPort());
                          		                           				      const char *port = (const char *) portBuffer;
                          		                           				      if (_gpsd_connected) {
                          		                           				        (void) gps_stream(&_gps_data, WATCH_DISABLE, NULL);
                          		                           				        gps_close (&_gps_data);
                          		                           				        _gpsd_connected = false;
                          		                           				      }
                          		                           				      if (gps_open(server, port, &_gps_data) == 0) {
                          		                           				        _gpsd_connected = true;
                          		                           				        (void) gps_stream(&_gps_data, WATCH_ENABLE | WATCH_JSON, NULL);
                          		                           				        IBRCOMMON_LOGGER_TAG(GpsdClient::TAG, info) << "Connected to gpsd at " << server << ":" << port << IBRCOMMON_LOGGER_ENDL;
                          		                           				      } else {
                          		                           				        IBRCOMMON_LOGGER_TAG(GpsdClient::TAG, error) << "Failed to connect to gpsd at " << server << ":" << port << IBRCOMMON_LOGGER_ENDL;
                          		                           				      }
                          		                           				    }

                          		                           				    void GpsdClient::componentUp() throw () {
                          		                           				      onConfigurationChanged(dtn::daemon::Configuration::getInstance());
                          		                           				    }

                          		                           				    void GpsdClient::componentRun() throw () {
                          		                           				      _noFixCounter = 0;
                          		                           				      dtn::data::Position position;
                          		                           				      while (! _shutdown)
                          		                           				      {
                          		                           				        {
                          		                           				          if (! _gpsd_connected)
                          		                           				          {
                          		                           				            _noFixCounter = 0;
                          		                           				            reconnectGpsd(dtn::daemon::Configuration::getInstance());
                          		                           				            if (! _gpsd_connected)
                          		                           				              Thread::sleep (1000);
                          		                           				          }
                          		                           				          else
                          		                           				          {
                          		                           				            int readRet = gps_read (&_gps_data);
                          		                           				            if (readRet < 0)
                          		                           				            {
                          		                           				              _noFixCounter = 0;
                          		                           				              IBRCOMMON_LOGGER_TAG(GpsdClient::TAG, error) << "Error in gps_read; attempting reconnect." << IBRCOMMON_LOGGER_ENDL;
                          		                           				              (void) gps_stream(&_gps_data, WATCH_DISABLE, NULL);
                          		                           				              gps_close (&_gps_data);
                          		                           				              _gpsd_connected = false;
                          		                           				              Thread::sleep (1000);
                          		                           				            }
                          		                           				            else if (readRet == 0 || (! _gps_data.set) || (_gps_data.status == 0))
                          		                           				            {
                          		                           				              if (_noFixCounter++ > 100)
                          		                           				              {
                          		                           				                IBRCOMMON_LOGGER_TAG(GpsdClient::TAG, warning) << "Failed to obtain GPS fix for over 10 seconds." << IBRCOMMON_LOGGER_ENDL;
                          		                           				                _noFixCounter = 0;
                          		                           				              }
                          		                           				              Thread::sleep (100);
                          		                           				            }

                          		                           				            else
                          		                           				            {
                          		                           				              _noFixCounter = 0;
                          		                           				              const double lat = _gps_data.fix.latitude;
                          		                           				              const double lon = _gps_data.fix.longitude;
                          		                           				              const double alt = _gps_data.fix.altitude;
                          		                           				              const double speed=_gps_data.fix.speed;
                          		                           				              const double climb=gps_data.fix.climb;
                          		                           				              const double heading=  gps_data.fix.track;
                          		                           				                long int time=gps_data.fix.time;

                          		                           				              IBRCOMMON_LOGGER_DEBUG_TAG(GpsdClient::TAG, 20) << "New fix: [lat=" << lat << ",lon=" << lon << ",alt=" << alt <<"speed"<<speed<<"climb"<<climb<<"heading"<<heading<<"time" "]"
                          		                           				                      << IBRCOMMON_LOGGER_ENDL;
                          		                           				              _gps_data.status &= ~(LATLON_SET | ALTITUDE_SET);
                          		                           				              position.setPosition(lat, lon, alt,speed,climb,heading,time);
                          		                           				              PositionEvent::raise (position, POSITION_FIX);
                          		                           				            }


                          		                           				          }
                          		                           				     double getDistance(double,double,double,double,double,double)
                          		                           				     {
                          		                           				    	 }
                          		                           				     }
                          		                           				     double getSpeed(double,double,double,double,double,double)
                          		                           				     {
                          		                           				    	 }
                          		                           				     }
                          		                           				          }
                          		                           				      }






                          		                           				    /*
                         gps_read::gps_read():speed(0),distance(0),climb(0),latitude_displacement(0),longitude_displacement(0),altitude_displacement(0)
                         {

                         }
                            gps_read::gps_read(values1)
                           {
                            	//val11 values;


                            	values1.val1 =this->gps_values(values1).val1;
                            	values1.val2 =this->gps_values(values1).val2;
                            	values1.val3 =this->gps_values(values1).val3;
                            	values1.val4 =this->gps_values(values1).val4;
                            	values1.val5 =this->gps_values(values1).val5;
                            	values1.val6 =this->gps_values(values1).val6;
                            	values1.val7 =this->gps_values(values1).val7;

                           }

                            gps_read::~gps_read()
                            {

                            }

                                val11 gps_read:: gps_values(values1)

                              {

                                	ibrcommon::MutexLock l(_gpsd_read_mutex);

                                try
                              {
                              gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);



                              if (gps_rec.stream(WATCH_ENABLE|WATCH_JSON) == NULL)

                                 {
                             IBRCOMMON_LOGGER_DEBUG_TAG(gps_read::TAG,error)<<"No GPSD running"<<  IBRCOMMON_LOGGER_ENDL;
                                   values1.val1=0;
                                  values1.val2=0;
                                  values1.val3=0;
                                  values1.val4=0;
                                  values1.val5=0;
                                   values1.val6=0;
                                   values1.val7=0;

                                    }
                                 else  {
                                	 //GPSEvent::raise (_gps_read, GPS_FIXED);
                                	 GPSEvent::raise (_gps_read,gps_action);
                                  struct gps_data_t* newdata;
                    values1.val1=newdata->fix.latitude, values1.val2=newdata->fix.longitude, values1.val3=newdata->fix.altitude, values1.val4=newdata->fix.speed, values1.val5=newdata->fix.track, values1.val6=newdata->fix.climb,
values1.val7=newdata->fix.time;
                    IBRCOMMON_LOGGER_DEBUG_TAG(gps_read::TAG,info)<<"GPSD is running" <<  IBRCOMMON_LOGGER_ENDL;
                                  }

                                      }
                                    catch(std::exception)
                                    {
                       IBRCOMMON_LOGGER_DEBUG_TAG(gps_read::TAG,error)<<"GPSD read error" <<  IBRCOMMON_LOGGER_ENDL;
                                       }

                                   return values1;
                                        }

                                //void gps_read::onConfigurationChanged(values1) throw () {
                                     //gps_values(values1);
                                  // }
                                /*
                                void gps_read::componentUp() throw ()
		{
                                	onConfigurationChanged(dtn::daemon::Configuration::getInstance());
		}
/////////////////////////////////////////////////////////////////////////////////////////////////
*//*
                                        double& gps_read::calculate_speed(values1,val12 values2)
                                        {
	                                   double theta;

                                               altitude_displacement=values1.val3-values2.val3;
                                                  theta=values1.val5-values2.val5;
                                                   climb=values1.val6-values2.val6;
                                                  if( theta<=180){
                                                      speed =sqrt(pow(values1.val4,2)+pow(values2.val4,2)+pow(climb,2)-2*values1.val4*values2.val4*cos(pi*theta/180));
                                                  }
                                                  else
                                                  {
                                                speed =sqrt(pow(values1.val4,2)+pow(values2.val4,2)+pow(climb,2)-2*values1.val4*values2.val4*abs(cos(pi*theta/180)));
                                             }
                                      return speed;
                                       }

                                                 double& gps_read::calculate_distance( values1, val12 values2)//const
                                                 {


                                                     latitude_displacement=abs(values1.val1 - values2.val1) *(pi / 180) * polar_radius;
                                                     longitude_displacement=abs(values1.val2 - values2.val2)*(pi / 180 )* (equatorial_radius * cos( (pi*values1.val1/180 + pi*values2.val2/180) / 2 ));
                                                     altitude_displacement=values1.val3-values2.val3;
                                                     distance=sqrt(pow(latitude_displacement,2)+ pow(altitude_displacement,2)+pow(longitude_displacement,2));
                                                      return distance;
	                                                        }


                                                   val17& gps_read::operator=(val17& values7)
                                                     {

	                                                    gps_read   _gps_read;
	                                                           val11 values1;
                                                                 values7.val1=0;
                                                             values7.val2=0;
                                                             values7.val3=0;
                                                             values7.val4=0;
                                                             values7.val5=0;
                                                             values7.val6=0;
                                                             values7.val7=0;
	                                                         values7.val1=_gps_read.gps_values(values1).val1;
	                                                          values7.val2=_gps_read.gps_values(values1).val2;
	                                                          values7.val3=_gps_read.gps_values(values1).val3;
	                                                         values7.val4=_gps_read.gps_values(values1).val4;
	                                                         values7.val5=_gps_read.gps_values(values1).val5;
	                                                          values7.val6=_gps_read.gps_values(values1).val6;
	                                                          values7.val7=_gps_read.gps_values(values1).val7;
                                                              }
                                                      val11& gps_read::operator=(values1)
                                                       {

	                                                 gps_read   _gps_read;
                                                     values1.val1=0;
                                       values1.val2=0;
                                       values1.val3=0;
                                       values1.val4=0;
                                       values1.val5=0;
                                       values1.val6=0;
                                       values1.val7=0;

                                        values1.val1=_gps_read.gps_values(values1).val1;
                                       	values1.val2=_gps_read.gps_values(values1).val2;
                                       	values1.val3=_gps_read.gps_values(values1).val4;
                                       	values1.val4=_gps_read.gps_values(values1).val4;
                                       	values1.val5=_gps_read.gps_values(values1).val5;
                                       	values1.val6=_gps_read.gps_values(values1).val6;
                                       	values1.val7=_gps_read.gps_values(values1).val7;

                                       }
                                                      const std::string GPSEvent::TAG = "GPSEvent";
                          GPSEvent::GPSEvent(const gps_read &gpsread, const GPSAction action)
                                              	: _gps_read(gpsread), gps_action(action)
                                              {

                                            switch (action)
                                            {
                                             	case GPS_NOT_RUNNING :

                                             		IBRCOMMON_LOGGER_DEBUG_TAG(GPSEvent::TAG,20)<<"No GPSD running"<<  IBRCOMMON_LOGGER_ENDL;


                                                  break;
                                                  case GPS_NOT_FIXED :

                                                	  IBRCOMMON_LOGGER_DEBUG_TAG(GPSEvent::TAG,20)<<"GPS read error" <<  IBRCOMMON_LOGGER_ENDL;
                                                    break;
                                                  case GPS_FIXED:

                                                	  IBRCOMMON_LOGGER_DEBUG_TAG(GPSEvent::TAG,20)<<"GPS is running" <<  IBRCOMMON_LOGGER_ENDL;

                                                    default:
                                                  	break;
                                                    	}
                                                       }

                       void GPSEvent::raise(const gps_read &gpsread, const GPSAction action)
                              		{
                                 	//dtn::core::EventDispatcher<GPSEvent>::queue( new GPSEvent(gpsread, action) );
                                 	dtn::core::EventDispatcher<GPSEvent>::raise( new GPSEvent(gpsread,action) );
                                    }

                             GPSEvent::~GPSEvent()
                                  {}

                      const gps_read& GPSEvent::getGPS() const
                        {
                        	return _gps_read;
                             }

                        GPSAction  GPSEvent::getGPSAction() const
                        {
                          			return gps_action;
                   		}

                    		const string GPSEvent::getName() const
                             {
                             	return "GPSEvent";
                             }
                    		       		string GPSEvent::getMessage() const
                                	{
                                switch (getGPSAction())
                                {
                                case GPS_NOT_RUNNING:
                                 return "GPS is not running .Please check";
                                 case GPS_NOT_FIXED :
                 				return "GPS not fixed yet. Please wait. Routing without getting fixed GPS data is not safe ";
                       			case GPS_FIXED:
                  				return "GPS got fixed . ";
                       			default:
                  				return "Can not figure out the GPS problem";
                       			}
                  			return "Can not figure out the GPS problem";
                     		}
	            }
                      }
*/
