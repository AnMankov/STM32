#include "median_filter.h"


TMedianFilter::TMedianFilter( int16_t _Stopper )
:
DataPtr( Buf ),
Small( { nullptr, _Stopper } ),
Big( { &Small, 0 } ),
Stopper( _Stopper )
{

}

TMedianFilter::~TMedianFilter()
{

}

int16_t TMedianFilter::process( int16_t Val )
{                                                      
  uint16_t i;                                                        
                                                                     
  if ( Val == Stopper )                                            
  {                                                                  
    Val = Stopper + 1;                                             /* No Stoppers allowed. */
  }                                                                  
                                                                     
  if ( (++DataPtr - Buf) >= Size )                 
  {                                                                  
    DataPtr = Buf;                                                 /* Increment and wrap data in Ptrer.*/
  }                                                                  
                                                                     
  DataPtr->Val = Val;                                              /* Copy in new Val */
  Successor = DataPtr->Ptr;                                        /* Save Ptrer to old Val's Successor */
  Median = &Big;                                                   /* Median initially to first in chain */
  Scanold = nullptr;                                               /* Scanold initially nullptr. */
  Scan = &Big;                                                     /* Ptrs to Ptrer to first (largest) Val in chain */
                                                                     
                                                                     
  if ( Scan->Ptr == DataPtr )                                     
  {                                                                  
   Scan->Ptr = Successor;                                          /* Handle chain-out of first item in chain as special case */
  }                                                                  
													                  
  Scanold = Scan;                                                  /* Save this Ptrer and */
  Scan = Scan->Ptr ;                                               /* step down chain */
                                                                     
  for ( i = 0 ; i < Size; ++i )                                    /* Loop through the chain, normal loop exit via break. */
  {                                                                
    /* Handle odd-numbered item in chain */                        
    if ( Scan->Ptr == DataPtr )                                 
    {                                                              
      Scan->Ptr = Successor;                                       /* Chain out the old Val.*/
    }                                                                
                                                                     
    if ( Scan->Val < Val )                                         /* If Val is larger than Scanned Val,*/
    {                                                                 
      DataPtr->Ptr = Scanold->Ptr;                                 /* Chain it in here. */
      Scanold->Ptr = DataPtr;                                      /* Mark it chained in. */
      Val = Stopper;
    };
    
    /* Step Median Ptrer down chain after doing odd-numbered element */
    Median = Median->Ptr;                                          /* Step Median Ptrer. */
    if ( Scan == &Small )                                            
    {                                                                
      break;                                                       /* Break at end of chain */
    }                                                                
	                                                                 
    Scanold = Scan;                                                /* Save this Ptrer and */
    Scan = Scan->Ptr;                                              /* step down chain */
    
    /* Handle even-numbered item in chain. */
    if ( Scan->Ptr == DataPtr )
    {
      Scan->Ptr = Successor;
    }
    
    if ( Scan->Val < Val )
    {
      DataPtr->Ptr = Scanold->Ptr;
      Scanold->Ptr = DataPtr;
      Val = Stopper;
    }
    
    if ( Scan == &Small )
    {
      break;
    }
    
    Scanold = Scan;
    Scan = Scan->Ptr;
  }
 
  return Median->Val;
}
