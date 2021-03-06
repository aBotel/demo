#include "mcp3008Spi.h"
#include "vector.h"
using namespace std;
/**********************************************************
 * spiOpen() :function is called by the constructor.
 * It is responsible for opening the spidev device 
 * "devspi" and then setting up the spidev interface.
 * private member variables are used to configure spidev.
 * They must be set appropriately by constructor before calling
 * this function. 
 * *********************************************************/
int mcp3008Spi::spiOpen(std::string devspi){
    int statusVal = -1;
    this->spifd = open(devspi.c_str(), O_RDWR);
    if(this->spifd < 0){
        perror("could not open SPI device");
        exit(1);
    }
 
    statusVal = ioctl (this->spifd, SPI_IOC_WR_MODE, &(this->mode));
    if(statusVal < 0){
        perror("Could not set SPIMode (WR)...ioctl fail");
        exit(1);
    }
 
    statusVal = ioctl (this->spifd, SPI_IOC_RD_MODE, &(this->mode)); 
    if(statusVal < 0) {
      perror("Could not set SPIMode (RD)...ioctl fail");
      exit(1);
    }
 
    statusVal = ioctl (this->spifd, SPI_IOC_WR_BITS_PER_WORD, &(this->bitsPerWord));
    if(statusVal < 0) {
      perror("Could not set SPI bitsPerWord (WR)...ioctl fail");
      exit(1);
    }
 
    statusVal = ioctl (this->spifd, SPI_IOC_RD_BITS_PER_WORD, &(this->bitsPerWord));
    if(statusVal < 0) {
      perror("Could not set SPI bitsPerWord(RD)...ioctl fail");
      exit(1);
    }   
 
    statusVal = ioctl (this->spifd, SPI_IOC_WR_MAX_SPEED_HZ, &(this->speed));     
    if(statusVal < 0) {
      perror("Could not set SPI speed (WR)...ioctl fail");
      exit(1);
    } 
 
    statusVal = ioctl (this->spifd, SPI_IOC_RD_MAX_SPEED_HZ, &(this->speed));     
    if(statusVal < 0) {
      perror("Could not set SPI speed (RD)...ioctl fail");
      exit(1);
    }
    return statusVal;
}
 
/***********************************************************
 * spiClose(): Responsible for closing the spidev interface.
 * Called in destructor
 * *********************************************************/
 
int mcp3008Spi::spiClose(){
    int statusVal = -1;
    statusVal = close(this->spifd);
        if(statusVal < 0) {
      perror("Could not close SPI device");
      exit(1);
    }
    return statusVal;
}
 
/********************************************************************
 * This function writes data "data" of length "length" to the spidev
 * device. Data shifted in from the spidev device is saved back into 
 * "data". 
 * ******************************************************************/
int mcp3008Spi::spiWriteRead( unsigned char *data, int length)
{
	struct spi_ioc_transfer spi[length];
	int i = 0; 
	int retVal = -1;  
 
	// one spi transfer for each byte
 
	for (i = 0 ; i < length ; i++){
	 
		spi[i].tx_buf        = (unsigned long)(data + i); // transmit from "data"
		spi[i].rx_buf        = (unsigned long)(data + i) ; // receive into "data"
		spi[i].len           = sizeof(*(data + i)) ;
		spi[i].delay_usecs   = 0 ; 
		spi[i].speed_hz      = this->speed ;
		spi[i].bits_per_word = this->bitsPerWord ;
		spi[i].cs_change = 0;
	}
	 
	retVal = ioctl (this->spifd, SPI_IOC_MESSAGE(length), &spi) ;
 
	if(retVal < 0){
		perror("Problem transmitting spi data..ioctl");
		exit(1);
	}
 
	return retVal;
 
}
 
float* mcp3008Spi::mcp3008_Scan(int devNum)
{
  float *force_array = (float*)malloc(sizeof(float)*16);
	int a2dVal = 0; 
  float a2dValf = 0;
  float a2dValc;
  int a2dChannel = 0;
	unsigned char data[3];	
	////cout << "A2D Num"<<devNum<<": ";
	for (a2dChannel=0;a2dChannel<8;a2dChannel++)
    {        
		data[0] = 1;  //  first byte transmitted -> start bit
		data[1] = 0b10000000 |( ((a2dChannel & 7) << 4)); // second byte transmitted -> (SGL/DIF = 1)
		data[2] = 0; // third byte transmitted....don't care
	 
		this->spiWriteRead(data, sizeof(data) );
	 
		a2dVal = 0;
        a2dVal = (data[1]<< 8) & 0b1100000000; //merge data[1] & data[2] to get result
        a2dVal |=  (data[2] & 0xff);
       
      //a2dValf = ((244.22311957*(a2dVal*5/1023)*(a2dVal*(5/1023)))+(817.22895852*a2dVal*5/1023) - 129.26615357)*(9.81/1000);
      
      a2dValc = a2dVal*(4.58/1023.0);
      
      a2dValf = 412.73707207*a2dValc*a2dValc+1076.3672085*a2dValc-111.1689124;
      
      a2dValf *= (9.81/1000.0);
      
      if(a2dValf < 0.0){
          a2dValf = 0.0;
      }
     // if((a2dChannel == 0) || (a2dChannel == 1) ){
     // a2dValf *= 3.421709*0.60108;
    //  }
       
	  force_array[a2dChannel] = a2dValf;
    
   
    
      
		//cout << "chnl"<<a2dChannel<<"=" << a2dValf << "   " << a2dVal<<std::endl << std::flush;
	}
	//cout << endl;
	a2dChannel = 0;
  return force_array;
} 


/*************************************************
 * Default constructor. Set member variables to
 * default values and then call spiOpen()
 * ***********************************************/
 
mcp3008Spi::mcp3008Spi(){
    this->mode = SPI_MODE_0 ; 
    this->bitsPerWord = 8;
    this->speed = 1000000;
    this->spifd = -1;
 
    this->spiOpen(std::string("/dev/spidev0.0"));
 
    }
 
/*************************************************
 * overloaded constructor. let user set member variables to
 * and then call spiOpen()
 * ***********************************************/
mcp3008Spi::mcp3008Spi(std::string devspi, unsigned char spiMode, unsigned int spiSpeed, unsigned char spibitsPerWord){
    this->mode = spiMode ; 
    this->bitsPerWord = spibitsPerWord;
    this->speed = spiSpeed;
    this->spifd = -1;
 
    this->spiOpen(devspi);
 
}
 
/**********************************************
 * Destructor: calls spiClose()
 * ********************************************/
mcp3008Spi::~mcp3008Spi(){
    this->spiClose();
}
