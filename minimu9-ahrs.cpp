
#include <errno.h>
#include <termios.h>


#include "version.h"
#include "vector.h"
#include "MinIMU9.h"
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <system_error>
#include <boost/program_options.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include "mcp3008Spi.h"
#include <curl/curl.h>
#include <array>
namespace opts = boost::program_options;

// TODO: print warning if accelerometer magnitude is not close to 1 when starting up

// An Euler angle could take 8 chars: -234.678, but usually we only need 6.
float field_width = 6;
char* BASE_URL;
char* COMM_MODE;
int fd1, wr;
#define FLOAT_FORMAT std::fixed << std::setprecision(3) << std::setw(field_width)

std::ostream & operator << (std::ostream & os, const vector & vector)
{
    return os << FLOAT_FORMAT << vector(0) << ' '
              << FLOAT_FORMAT << vector(1) << ' '
              << FLOAT_FORMAT << vector(2);
}

std::ostream & operator << (std::ostream & os, const matrix & matrix)
{
    return os << (vector)matrix.row(0) << ' '
              << (vector)matrix.row(1) << ' '
              << (vector)matrix.row(2);
}

std::ostream & operator << (std::ostream & os, const quaternion & quat)
{
    return os << FLOAT_FORMAT << quat.w() << ' '
              << FLOAT_FORMAT << quat.x() << ' '
              << FLOAT_FORMAT << quat.y() << ' '
              << FLOAT_FORMAT << quat.z();
}

typedef void rotation_output_function(quaternion& rotation);

void output_quaternion(quaternion & rotation)
{
    //std::cout << rotation;
}

void output_matrix(quaternion & rotation)
{
    //std::cout << rotation.toRotationMatrix();
}

void output_euler(quaternion & rotation)
{
    //std::cout << (vector)(rotation.toRotationMatrix().eulerAngles(2, 1, 0) * (180 / M_PI));
}

int millis()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

void streamRawValues(IMU& imu)
{
    imu.enable();
    while(1)
    {
        imu.read();
        printf("%7d %7d %7d  %7d %7d %7d  %7d %7d %7d\n",
               imu.raw_m[0], imu.raw_m[1], imu.raw_m[2],
               imu.raw_a[0], imu.raw_a[1], imu.raw_a[2],
               imu.raw_g[0], imu.raw_g[1], imu.raw_g[2]
        );
        usleep(20*1000);
    }
}

//! Uses the acceleration and magnetic field readings from the compass
// to get a noisy estimate of the current rotation matrix.
// This function is where we define the coordinate system we are using
// for the ground coords:  North, East, Down.
matrix rotationFromCompass(const vector& acceleration, const vector& magnetic_field)
{
    vector down = -acceleration;     // usually true
    vector east = down.cross(magnetic_field); // actually it's magnetic east
    vector north = east.cross(down);

    east.normalize();
    north.normalize();
    down.normalize();

    matrix r;
    r.row(0) = north;
    r.row(1) = east;
    r.row(2) = down;
    return r;
}

typedef void fuse_function(quaternion& rotation, quaternion& velocity, float dt, const vector& angular_velocity,
                  const vector& acceleration, const vector& magnetic_field);

void fuse_compass_only(quaternion& rotation, float dt, const vector& angular_velocity,
  const vector& acceleration, const vector& magnetic_field)
{
    // Implicit conversion of rotation matrix to quaternion.
    rotation = rotationFromCompass(acceleration, magnetic_field);
}

// Uses the given angular velocity and time interval to calculate
// a rotation and applies that rotation to the given quaternion.
// w is angular velocity in radians per second.
// dt is the time.
void rotate(quaternion& rotation, const vector& w, float dt)
{
    // Multiply by first order approximation of the
    // quaternion representing this rotation.
    rotation *= quaternion(1, w(0)*dt/2, w(1)*dt/2, w(2)*dt/2);
    rotation.normalize();
}

void rotate_velocity(quaternion& velocity, const vector& w, float dt, vector& velocity_top)
{
    float shaft_length = 0.75; 
    // Multiply by first order approximation of the
    // quaternion representing this rotation.
    velocity *= quaternion(1, w(0)*dt/2, w(1)*dt/2, w(2)*dt/2);   
    velocity_top[0] = shaft_length * w(0); 
    velocity_top[1] = shaft_length * w(1); 
    velocity_top[2] = shaft_length * w(2); 
}

void fuse_gyro_only(quaternion& rotation, float dt, const vector& angular_velocity,
  const vector& acceleration, const vector& magnetic_field)
{
    rotate(rotation, angular_velocity, dt);
}

void fuse_default(quaternion& rotation, quaternion& velocity, float dt, const vector& angular_velocity,
  const vector& acceleration, const vector& magnetic_field, vector& velocity_top)
{
    vector correction = vector(0, 0, 0);

    if (abs(acceleration.norm() - 1) <= 0.3)
    {
        // The magnetidude of acceleration is close to 1 g, so
        // it might be pointing up and we can do drift correction.

        const float correction_strength = 1;

        matrix rotationCompass = rotationFromCompass(acceleration, magnetic_field);
        matrix rotationMatrix = rotation.toRotationMatrix();

        correction = (
            rotationCompass.row(0).cross(rotationMatrix.row(0)) +
            rotationCompass.row(1).cross(rotationMatrix.row(1)) +
            rotationCompass.row(2).cross(rotationMatrix.row(2))
          ) * correction_strength;

    }

    rotate(rotation, angular_velocity + correction, dt);
	  rotate_velocity(velocity, angular_velocity + correction, dt, velocity_top);
    
}

void bluetooth_request(char* buffer)
{
    wr=write(fd1,buffer,strlen(buffer));
}

void curl_request(char* buffer, size_t size, char* url)
{
CURL *curl;
CURLcode res;
char final_url[100];
strncpy(final_url, BASE_URL, sizeof(char)*strlen(BASE_URL));
strcpy (final_url+sizeof(char)*strlen(BASE_URL), "motion/api/v1/");
strcpy (final_url+strlen(final_url), url);
curl = curl_easy_init();
if(curl){
curl_easy_setopt(curl,CURLOPT_READDATA,buffer);
curl_easy_setopt(curl, CURLOPT_PUT, 1L);
curl_easy_setopt(curl, CURLOPT_URL, final_url);
curl_easy_setopt(curl,CURLOPT_BUFFERSIZE,size);
res = curl_easy_perform(curl);
        //std::cout << BASE_URL << "  " <<final_url << "   " << buffer  << "    "<< std::endl << std::flush;

	if(res!=CURLE_OK)
	{
		//std::cout<<"STuff failed no curl!"<<std::endl;
	}
curl_easy_cleanup(curl);

}//if
}

void send_force(float* force)
{
  //std::cout << "    " << force[0] << "    " << force[9] <<std::endl << std::flush;
	char data_buf[150];
  sprintf(data_buf, "{\'1\':%f,\'2\':%f,\'3\':%f,\'4\':%f,\'5\':%f,\'6\':%f,\'7\':%f,\'8\':%f,\'9\':%f,\'10\':%f}", force[0], force[1], force[2], force[3], force[4], force[5], force[6], force[7], force[8], force[9]);
  char url[20];
	sprintf(url, "force");
  //printf("The value of COMM_MODE a char is %s",COMM_MODE);
  if(*COMM_MODE == '1')
  {
  //printf("COMM_MODE is in serial mode");
	curl_request(data_buf,strlen(data_buf),url);
  }
  
  else if(*COMM_MODE == '0')
  {
  bluetooth_request(data_buf);
  }
 
	
}

void send_velo(quaternion& velocity)
{
	char data_buf[50];
  sprintf(data_buf, "{\'x\':%f,\'y\':%f,\'z\':%f}", velocity.x(), velocity.y(), velocity.z());
	char url[20];
	sprintf(url, "velo");
  if(*COMM_MODE == '1')
  {
	curl_request(data_buf,strlen(data_buf),url);
  }
  
  else if(*COMM_MODE == '0')
  {
  bluetooth_request(data_buf);
  }
	
}

void send_velo_top(vector& velocity_top)
{
	char data_buf[50];
  sprintf(data_buf, "{\'x\':%f,\'y\':%f,\'z\':%f}", velocity_top[0], velocity_top[1], velocity_top[2]);
	char url[20];
	sprintf(url, "velo_top");
  if(*COMM_MODE == '1')
  {
	curl_request(data_buf,strlen(data_buf),url);
  }
   
  else if(*COMM_MODE == '0')
  {
  bluetooth_request(data_buf);
  }
	
}

void send_power(float& power)
{
	char data_buf[50];
  sprintf(data_buf, "{\'power\':%f}", power);
	char url[20];
	sprintf(url, "power");
  if(*COMM_MODE == '1')
  {
	curl_request(data_buf,strlen(data_buf),url);
  }
  
  else if(*COMM_MODE == '0')
  {
  bluetooth_request(data_buf);
  }
	
}

void send_accel(const vector& acceleration_corrected, float dt)
{
	char data_buf[50];
        sprintf(data_buf, "{\'x\':%f,\'y\':%f,\'z\':%f, \'dt\':%f}", acceleration_corrected[0], acceleration_corrected[1], acceleration_corrected[2], dt);
	char url[20];
	sprintf(url, "accel");
   if(*COMM_MODE == '1')
  {
	curl_request(data_buf,strlen(data_buf),url);
  }
  
  else if(*COMM_MODE == '0')
  {
  bluetooth_request(data_buf);
  }
	
}
void ahrs(IMU & imu, rotation_output_function * output)
{
		mcp3008Spi a2d1("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);
                mcp3008Spi a2d2("/dev/spidev0.1", SPI_MODE_0, 1000000, 8);

  fd1=open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd1 == -1 )
    {
             perror("open_port: Unable to open /dev/ttyS0 – ");
    }             
     else
    {
             fcntl(fd1, F_SETFL,0);
             printf("Port 1 has been sucessfully opened and %d is the file description\n",fd1);
    }
  
	curl_global_init(CURL_GLOBAL_ALL);
    imu.loadCalibration();
    imu.enable();
    imu.measureOffsets();
    int count = 0;
    // The quaternion that can convert a vector in body coordinates
    // to ground coordinates when it its changed to a matrix.
    quaternion rotation = quaternion::Identity();
    quaternion velocity = quaternion::Identity();
    quaternion velocity_old = quaternion::Identity();
    quaternion velocity2 = quaternion::Identity();
    quaternion velocity_temp = quaternion::Identity();
    vector acceleration_corrected;
    vector velocity_top;
    float* force1;
    float* force2;
    float power;
    float velocity2_mag;
    float velocity_mag;
    float velocity_old_mag;
    matrix rotation_matrix;
    int start = millis(); // truncate 64-bit return value
    
    int fd1;
    int wr,rd,nbytes,tries;
    

    
    
    while(1)
    {
        int last_start = start;
        start = millis();
        float dt = (start-last_start)/1000.0;
        if (dt < 0){ throw std::runtime_error("Time went backwards."); }

        vector angular_velocity = imu.readGyro();
        vector acceleration = imu.readAcc();
        vector magnetic_field = imu.readMag();
	velocity_old = velocity_temp;
  fuse_default(rotation, velocity, dt, angular_velocity, acceleration, magnetic_field, velocity_top);
	rotation_matrix = rotation.toRotationMatrix();
	velocity_temp = velocity;
	vector gravity_vector = (vector)rotation_matrix.row(2);
  acceleration_corrected[0] = acceleration[0] + gravity_vector[0];
	acceleration_corrected[1] = acceleration[1] + gravity_vector[1];
	acceleration_corrected[2] = acceleration[2] + gravity_vector[2];
	velocity2.x() = acceleration_corrected[0]*9.81*dt + velocity.x();
        velocity2.y() = acceleration_corrected[1]*9.81*dt + velocity.y(); 
	velocity2.z() = acceleration_corrected[2]*9.81*dt + velocity.z();  
	velocity2_mag = sqrtf(velocity2.x()*velocity2.x() + velocity2.y()*velocity2.y() + velocity2.z()*velocity2.z());
	velocity_mag = sqrtf(velocity.x()*velocity.x() + velocity.y()*velocity.y() + velocity.z()*velocity.z());
	velocity_old_mag = sqrtf(velocity_old.x()*velocity_old.x() + velocity_old.y()*velocity_old.y() +  velocity_old.z()*velocity_old.z());
        
        if( velocity_mag < velocity2_mag && velocity_mag < velocity_old_mag){
        velocity.x() = 0;
	velocity.y() = 0;
	velocity.z() = 0;
	velocity2.x() = acceleration_corrected[0]*9.81*dt + velocity.x();
	velocity2.y() = acceleration_corrected[1]*9.81*dt + velocity.y();
	velocity2.z() = acceleration_corrected[2]*9.81*dt + velocity.z();
	}
        velocity.x() = velocity2.x();
        velocity.y() = velocity2.y();
        velocity.z() = velocity2.z();
 
	if (count < 100){
        velocity.x() = 0;
        velocity.y() = 0;
        velocity.z() = 0;
        count += 1;
        }
	    
  force1 = a2d1.mcp3008_Scan(1);
	force2 = a2d2.mcp3008_Scan(2);
  force1[8] = force2[0];
  force1[9] = force2[1];
     
   
  //std::cout << "    " << force1[0] << "    " << force1[9] <<std::endl << std::flush;
  /*for(int x = 0; x < 10; x++){
  force1[x] =((244.22311957*force1[x]*force1[x]*(5/1023)*(5/1023))+(817.22895852*force1[x]*5/1023) - 129.26615357)*(9.81/1000);
  }*/
    
  
  //send data
  send_accel(acceleration_corrected, dt);		
  send_velo(velocity);
  send_velo_top(velocity_top);
  send_force(force1);  
  
  
  //CODE FOR POWER CALCULATIONS
  
  power = 1337;
  send_power(power);
  
  
 
        //std::cout << "    " << acceleration << "    " << velocity << "    " << dt << std::endl << std::flush;
        // Ensure that each iteration of the loop takes at least 20 ms.
        while(millis() - start < 10)
        {
            usleep(1000);
        }
        
        
    }
curl_global_cleanup();
}

int main(int argc, char *argv[])
{
	
    try
    {
		//mcp3008Spi *ptr = &a2d;
		//int adxl1_addr = 0x53;
		
		// Define what all the command-line parameters are.
        std::string mode, output_mode, i2cDevice, url, comm_mode;
        opts::options_description desc("Allowed options");
        desc.add_options()
            ("help,h", "produce help message")
            ("version,v", "print version number")
            ("i2c-bus,b", opts::value<std::string>(&i2cDevice)->default_value("/dev/i2c-0"),
             "i2c-bus the IMU is connected to")
            ("mode", opts::value<std::string>(&mode)->default_value("normal"),
             "specifies what algorithm to use.\n"
             "normal: Fuse compass and gyro.\n"
             "gyro-only:  Use only gyro (drifts).\n"
             "compass-only:  Use only compass (noisy).\n"
             "raw: Just print raw values from sensors.")
            ("output", opts::value<std::string>(&output_mode)->default_value("matrix"),
             "specifies how to output the orientation.\n"
             "matrix: Direction Cosine Matrix.\n"
             "quaternion: Quaternion.\n"
             "euler: Euler angles (yaw, pitch, roll).\n")
             ("url,u", opts::value<std::string>(&url)->default_value("http:/fydp.ngrok.com/"),
             "url: specifies base url to post http requests to.\n")
			       ("comm_mode,c", opts::value<std::string>(&comm_mode)->default_value("1"),
             "communication protocol  to use 1-> wifi 0 -> bluetooth\n")
            ;
        opts::variables_map options;
        opts::store(opts::command_line_parser(argc, argv).options(desc).run(), options);
        opts::notify(options);

        if(options.count("help"))
        {
            //std::cout << desc << std::endl;
            //std::cout << "For more information, run: man minimu9-ahrs" << std::endl;
            return 0;
        }
        
        if (options.count("version"))
        {
            //std::cout << VERSION << std::endl;
            return 0;
        }

        MinIMU9 imu(i2cDevice.c_str());

        rotation_output_function * output;

        // Figure out the output mode.
        if (output_mode == "matrix")
        {
            output = &output_matrix;
        }
        else if (output_mode == "quaternion")
        {
            output = &output_quaternion;
        }
        else if (output_mode == "euler")
        {
            field_width += 2;  // See comment above for field_width.
            output = &output_euler;
        }
        else
        {
            std::cerr << "Unknown output mode '" << output_mode << "'" << std::endl;
            return 1;
        }

        // Figure out the basic operating mode and start running.
        if (mode == "raw")
        {
            streamRawValues(imu);
        }
        else if (mode == "gyro-only")
        {
            //ahrs(imu, &fuse_gyro_only, output);
        }
        else if (mode == "compass-only")
        {



           // ahrs(imu, &fuse_compass_only, output);
        }
        else if (mode == "normal")
        {            
             BASE_URL = (char*)malloc(strlen(url.c_str())*sizeof(char));
             strcpy(BASE_URL,url.c_str());
             COMM_MODE = (char*)malloc(strlen(comm_mode.c_str())*sizeof(char));
             //COMM_MODE = (int)comm_mode;
             strcpy(COMM_MODE,comm_mode.c_str());
             std::cout << COMM_MODE << std::endl << std::flush;
            ahrs(imu, output);
        }
        else
        {
            std::cerr << "Unknown mode '" << mode << "'" << std::endl;
            return 1;
        }
        return 0;
    }
    catch(const std::system_error & error)
    {
        std::string what = error.what();
        const std::error_code & code = error.code();
        std::cerr << "Error: " << what << "  " << code.message() << " (" << code << ")" << std::endl;
        return 2;
    }
    catch(const opts::multiple_occurrences & error)
    {
        std::cerr << "Error: " << error.what() << " of " << error.get_option_name() << " option." << std::endl;
        return 1;
    }
    catch(const std::exception & error)    
    {
        std::cerr << "Error: " << error.what() << std::endl;
        return 9;
    }
}
