/*
 * gps_read.h
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
#ifndef GPS_READ_H_
#define GPS_READ_H_
#include <core/Event.h>
#include "core/Event.h"
#include <ibrcommon/Logger.h>
#include <iostream>
#include <string>
#include <libgpsmm.h>
#include <string>
#include<sstream>
#include<stdio.h> 
#include<cmath>
#include<cstdlib>
using namespace dtn::core;
namespace dtn
{
	namespace GPSClient
	{

	class Gpsd: public Configuration::Extension
				{
					friend class Configuration;
				protected:
					Gpsd();
					virtual ~Gpsd();
					void load(const ibrcommon::ConfigFile &conf);

				private:
					bool _enabled;
					std::string _host;
	                                unsigned int _port;

				public:
					/**
					 * @return True, if the gpsd service is enabled
					 */
					bool enabled() const;

					/**
					 * @return the host name of the gpsd
					 */
					std::string getHost() const;

					/**
					 * @return the port number of the gpsd
					 */
					unsigned int getPort() const;

				};
	class Position
		{

	                private:

	                  double _lat, _lon, _alt ,_speed,_climb,_heading,_time;

			public:

				Position();
				Position(double lat, double lon, double alt,double speed,double climb,double heading,long int time);
				~Position();

				double getLatitude() const;
	                        double getLongitude()const;
	                        double getAltitude()const;
	                        double getSpeed()const;
	                        double getClimb()const;
	                        double getHeading()const;
	                        double getTime()const;


	                        void setPosition (double lat, double lon, double alt,double speed,double climb,double heading,double time);
double getDistance(double,double,double,double,double,double);
double getSpeed(double,double,double,double,double,double);
				std::string toString() const;

			private:

			};
	enum PositionEventAction
			{
				POSITION_FIX = 0
			};

			class PositionEvent : public Event
			{
			public:
				virtual ~PositionEvent();

				PositionEventAction getAction() const;
				const dtn::data::Position& getPosition() const;
				const std::string getName() const;

				static void raise(const dtn::data::Position &position, const PositionEventAction action);

				std::string getMessage() const;

			private:
				PositionEvent(const dtn::data::Position &position, const PositionEventAction action);

				const dtn::data::Position m_position;
				const PositionEventAction m_action;
			};
			        class GpsdClient
			           : public dtn::daemon::IndependentComponent, public dtn::daemon::Configuration::OnChangeListener
			            {

			            public:

			              GpsdClient();
			              virtual ~GpsdClient();

			              static const std::string TAG;

			              virtual const std::string getName() const;

			              virtual void onConfigurationChanged(const dtn::daemon::Configuration &conf) throw ();

			            protected:
			              virtual void componentUp() throw ();
			              virtual void componentRun() throw ();
			              virtual void componentDown() throw ();
			              virtual void __cancellation() throw ();
			           private:

			             void reconnectGpsd(const dtn::daemon::Configuration &config) throw ();
			             ibrcommon::Mutex _gpsd_connection_mutex;
			             bool _gpsd_connected = false;
			             struct gps_data_t _gps_data;
			             bool _shutdown;
			             unsigned int _noFixCounter;
			            };
			           class GpsdClient
			              : public dtn::daemon::IndependentComponent, public dtn::daemon::Configuration::OnChangeListener
			               {

			               public:

			                 GpsdClient();
			                 virtual ~GpsdClient();

			                 static const std::string TAG;

			                 virtual const std::string getName() const;

			                 virtual void onConfigurationChanged(const dtn::daemon::Configuration &conf) throw ();

			               protected:
			                 virtual void componentUp() throw ();
			                 virtual void componentRun() throw ();
			                 virtual void componentDown() throw ();
			                 virtual void __cancellation() throw ();
			              private:

			                void reconnectGpsd(const dtn::daemon::Configuration &config) throw ();
			                ibrcommon::Mutex _gpsd_connection_mutex;
			                bool _gpsd_connected = false;
			                struct gps_data_t _gps_data;
			                bool _shutdown;
			                unsigned int _noFixCounter;

			               };


	}/* name space GPSClient*/
}/*namespace dtn*/
#endif /* GPS_READ.H */
/*
	struct val11{
	 	double val1;
	 	double val2;
	 	double  val3;
	 	double  val4;
	 	double val5;
	 	double  val6;
	 	double val7;

	   }values1;
	  struct val13{
	 	  float val1;
	 	  float val2;
	 	  float val3;
	 	  float val4;
	 	  float val5;
	 	  float val6;
	  double val7;
	  };
	  struct val12 {
	 double val1;
	  double  val2;
	  double val3;
	  double val4;
	  double val5;
	  double val6;
	  double val7;};

	  struct val17{
	 	float val1;
	  float val2;
	  float val3;
	  float val4;
	  float val5;
	  float val6;
	  double val7;};
 class gps_read
       {
		  static const std::string TAG;
public :

gps_read();
~gps_read();
  val11 gps_values(val11);
  //void onConfigurationChanged(const dtn::daemon::Configuration &config) throw ();
  void componentUp() throw () ;
 double& calculate_distance(val11,val12) ;
 double& calculate_speed( val11, val12) ;
 val11& operator=( val11&);
 val17& operator=( val17&);
   private:
 gps_read(const val11 );

   double distance;
   double speed;
   double latitude_displacement;
   double longitude_displacement;
   double altitude_displacement;
   double climb;
   ibrcommon::Mutex _gpsd_read_mutex;
   //const dtn::daemon::Configuration &_config ;

       };

 enum GPSAction
  		{
  			GPS_NOT_RUNNING = 0,
  			GPS_NOT_FIXED = 1,
  			GPS_FIXED = 2

  		};


  class GPSEvent :public gps_read,public Event
  {
	  static const std::string TAG;
  public:

  			virtual ~GPSEvent();

  			GPSAction getGPSAction() const;
  			const gps_read& getGPS() const;
  			const std::string getName() const;

  			std::string getMessage() const;

  			static void raise(const gps_read &gpsread, const GPSAction action);

  		private:
  			GPSEvent(const gps_read &gpsread, const GPSAction action);

  			const gps_read _gps_read;
  			const GPSAction gps_action;
  };
 } */
