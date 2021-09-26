#ifndef _RING_BUFFER_PLUS_
#define _RING_BUFFER_PLUS_

#define incomingBufferSize (512*64)

class RingBufferNPlus
{
  public:
    volatile uint8_t _aucBuffer[incomingBufferSize] ;
    volatile int _iHead ;
    volatile int _iTail ;

  public:
    RingBufferNPlus( void ) ;
    void store_char( uint8_t c ) ;
    void clear();
    int read_char();
    int read_chars(volatile char *dest, int numChars);
    int available();
    int availableForStore();
    int peek();
    bool isFull();

  private:
    volatile int nextIndex(volatile int index);
};

#endif
