#include "ax2550/ax2550.h"

#include <iostream>

#include <boost/algorithm/string.hpp>

using namespace ax2550;

using std::string;
using serial::Serial;

inline void defaultInfo(const string &msg) {
  std::cout << "AX2550 Info: " << msg << std::endl;
}

AX2550::AX2550 (string port)
: port_(""), seiral_port_(NULL), connected_(false), synched_(false),
serial_listener_(true)
{
  this->port_ = port;
  if (!this->port.empty()) {
    this->connect();
  }
  // Set default callbacks
  this->info = defaultInfo;
  this->watch_dog_callback = NULL;
}

AX2550::~AX2550 () {
  this->disconnect();
}

void
AX2550::connect (string port) {
  // Make sure we aren't already connected
  if (this->connected_) {
    THROW(ConnectionException, "already connected");
  }
  // If a port was passed in, set the interal one
  if (!port.empty()) {
    this->port_ = port;
  }
  // Check to see if the port is set to something
  if (this->port_.empty()) {
    THROW(ConnectionException, "serial port name is empty");
  }
  // Call disconnect to ensure we are in a clean state
  this->disconnect();
  // Setup the filters
  this->setupFilters_();
  // Setup the serial port
  this->serial_port_ = new Serial();
  this->serial_port_.setPort(this->port_);
  this->serial_port_.setBaudrate(9600);
  this->serial_port_.setParity(serial::parity_even);
  this->serial_port_.setStopbits(serial::stopbits_one);
  this->serial_port_.setBytesize(serial::sevenbits);
  this->serial_port_.setTimeout(250);
  // Open the serial port
  this->serial_port_.open();
  // Setup the serial listener
  this->serial_listener_.startListening(this->serial_port_);
  this->connected_ = true;
  // Synchronize with the motor controller
  this->sync_();
}

void
AX2550::disconnect () {
  this->connected_ = false;
  this->serial_listener_.stopListening();
  if (this->serial_port_ != NULL) {
    delete this->serial_port_;
    this->serial_port_ = NULL;
  }
}

bool
AX2550::issueCommand (const string &command, string &fail_why) {
  // Setup an echo filter
  BufferedFilterPtr echo_filt =
    this->serial_listener_.createBufferedFilter(exactly(command));
  this->serial_port_.write(command);
  // Listen for the echo
  if (echo_filt.wait(50).empty()) {
    fail_why = "failed to receive an echo";
    return false;
  }
  return true;
}

void
AX2550::move (double speed, double direction) {
  if(!this->connected) {
    THROW(CommandFailedException, "must be connected to move")
  }
  char *serial_buffer = new char[5];
  unsigned char speed_hex, direction_hex;
  string fail_why;
  // Create the speed command
  speed_hex = (unsigned char) (fabs(speed));
  if(speed < 0) {
    sprintf(serial_buffer, "!a%.2X\r", speed_hex);
  } else {
    sprintf(serial_buffer, "!A%.2X\r", speed_hex);
  }
  // Issue the speed command
  if (!this->issueCommand(string(serial_buffer), fail_why)) {
    THROW(CommandFailedException, fail_why);
  }
  // Listen for an ack or nak
  this->ack_nak_filt.clear();
  string result = this->ack_nak_filt.wait(100);
  if (result != "+") {
    if (result == "-") {
      THROW(CommandFailedException, "nak received, command failed");
    }
    THROW(CommandFailedException, "did not receive an ack or nak");
  }
  // Reset the buffer  
  delete[] serial_buffer;
  serial_buffer = new char[5];
  // Create the direction command
  direction_hex = (unsigned char) (fabs(direction));
  if(direction < 0) {
    sprintf(serial_buffer, "!b%.2X\r", direction_hex);
  } else {
    sprintf(serial_buffer, "!B%.2X\r", direction_hex);
  }
  // Issue the direction command
  if (!this->issueCommand(string(serial_buffer), fail_why)) {
    THROW(CommandFailedException, fail_why);
  }
  // Listen for an ack or nak
  this->ack_nak_filt.clear();
  result = this->ack_nak_filt.wait(100);
  if (result != "+") {
    if (result == "-") {
      THROW(CommandFailedException, "nak received, command failed");
    }
    THROW(CommandFailedException, "did not receive an ack or nak");
  }
}

void
AX2550::queryEncoders (long &encoder1, long &encoder2, bool relative) {
  // Check the count in the encoder filter, should be 0
  if (this->encoders_filt.count()) {
    stringstream ss;
    ss << "There were " << this->encoders_filt.count()
       << " orphaned encoder messages in the filter...";
    this->warn(ss.str());
  }
  // Clear the encoder queue
  this->encoders_filt.clear();
  // Query the first encoder
  string cmd, fail_why;
  if (relative) {
    cmd = "?q4\r";
  } else {
    cmd = "?q0\r";
  }
  encoder1 = this->queryEncoder_(cmd);
  // Query the second encoder
  string cmd, fail_why, result;
  if (relative) {
    cmd = "?q5\r";
  } else {
    cmd = "?q1\r";
  }
  encoder2 = this->queryEncoder_(cmd);
}

long
AX2550::queryEncoder_ (const string &cmd) {
  // Issue the command
  if (!this->issueCommand(cmd, fail_why)) {
    THROW(CommandFailedException, fail_why);
  }
  // Wait for the response
  string response = this->encoders_filt.wait(100);
  if (response.empty()) {
    THROW(CommandFailedException, "failed to receive a response from "+cmd);
  }
  // Parse the response
  char fillbyte;
  // Determine sign
  if(response.substr(0,1).find_first_of("01234567") != std::string::npos) {
    // Then positive
    fillbyte = '0';
  } else {
    // Then negative
    fillbyte = 'F';
  }
  // Add filler bytes
  size_t difference = 8 - response.length();
  string filler(difference, fillbyte)
  response.insert(0, fillbyte);
  // Convert to integer
  signed int encoder = 0;
  sscanf(response.c_str(), "%X", &encoder);
  return encoder;
}

void
AX2550::sync_ () {
  if (this->synched_)
    return;
  // Reset the motor controller
  this->serial_port_.write("%rrrrrr\r");
  // Wait for an R/C Message
  this->rc_msg_filter_.clear();
  if (this->rc_msg_filter_.wait(1000).empty()) {
    THROW(SynchronizationException,
      "did not receive an R/C message after reset");
  }
  // Write \r to the port until in serial mode
  BlockingFilter ok_filt(SerialListener::exactly("OK"));
  bool got_ok = false;
  for (int i = 0; i < 10; ++i) {
    this->serial_port_.write("\r");
    if (!ok_filt.wait(25).empty()) {
      got_ok = true;
      break;
    }
  }
  // Check to see if we ever got an OK
  if (!got_ok) {
    THROW(SynchronizationException, "failed to get into serial mode");
  }
  this->synched_ = true;
  this->info("Synchronized with the ax2550");
}

inline bool
isAnEncoderMsg (const string &token) {
  // If token[0] is any of 0123456789abcdefABDEF (hex)
  if (boost::algorithm::contains("0123456789abcdefABCDEF", token.substr(0,1)))
  {
    return true;
  }
  return false;
}

void
AX2550::watchDogCallback_ (const string &token) {
  if (this->watch_dog_callback != NULL) {
    this->watch_dog_callback();
  }
}

inline bool
isAckOrNak (const string &token) {
  if (boost::algorithm::contains(token, "+")
   || boost::algorithm::contains(token, "-"))
  {
    return true;
  }
  return false;
}

void
AX2550::setupFilters_ () {
  // Setup the encoder filter
  this->encoders_filt =
    this->serial_listener_.createBufferedFilter(isAnEncoderMsg);
  // Setup the watchdog filter
  this->watch_dog_filt = this->serial_listener_.createFilter(
    SerialListener::exactly("W"),
    boost::bind(&AX2550::watchDogCallback_, this, _1));
  // Setup ack/nak filter
  this->ack_nak_filt = this->serial_listener_.createBufferedFilter(isAckOrNak);
  // Setup R/C message filter
  this->rc_msg_filter_ = this->serial_listener_.createBufferedFilter(
    SerialListener::startsWith(":"));
}





