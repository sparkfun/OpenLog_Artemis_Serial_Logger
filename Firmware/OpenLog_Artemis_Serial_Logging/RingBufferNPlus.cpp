#include "RingBufferNPlus.h"

RingBufferNPlus::RingBufferNPlus( void )
{
  memset((void *)_aucBuffer, 0, incomingBufferSize ) ;
  clear();
}

void RingBufferNPlus::store_char( uint8_t c )
{
  int i = nextIndex(_iHead);

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if ( i != _iTail )
  {
    _aucBuffer[_iHead] = c ;
    _iHead = i ;
  }
}

void RingBufferNPlus::clear()
{
  _iHead = 0;
  _iTail = 0;
}

int RingBufferNPlus::read_char()
{
  if(_iTail == _iHead)
    return -1;

  uint8_t value = _aucBuffer[_iTail];
  _iTail = nextIndex(_iTail);

  return value;
}

int RingBufferNPlus::available()
{
  int delta = _iHead - _iTail;

  if(delta < 0)
    return incomingBufferSize + delta;
  else
    return delta;
}

int RingBufferNPlus::availableForStore()
{
  if (_iHead >= _iTail)
    return incomingBufferSize - 1 - _iHead + _iTail;
  else
    return _iTail - _iHead - 1;
}

int RingBufferNPlus::peek()
{
  if(_iTail == _iHead)
    return -1;

  return _aucBuffer[_iTail];
}

volatile int RingBufferNPlus::nextIndex(volatile int index)
{
  return (volatile int)((index + 1) % incomingBufferSize);
}

bool RingBufferNPlus::isFull()
{
  return (nextIndex(_iHead) == _iTail);
}

int RingBufferNPlus::read_chars(volatile char *dest, int numChars)
{
  if (available() < numChars) // Bail if the buffer contains less than numChars
    return -1;
  if ((_iTail + numChars) >= incomingBufferSize) // Check for wrap-around
  {
    // Wrap-around. Do two memcpy's
    memcpy((char *)dest, (const char *)&_aucBuffer[_iTail], (incomingBufferSize - _iTail));
    memcpy((char *)(dest + (incomingBufferSize - _iTail)), (const char *)&_aucBuffer[0], (numChars - (incomingBufferSize - _iTail)));
    _iTail = (numChars - (incomingBufferSize - _iTail));
  }
  else
  {
    // No wrap-around. Do a single memcpy
    memcpy((char *)dest, (const char *)&_aucBuffer[_iTail], numChars);
    _iTail += numChars;
  }
  return numChars;
}
