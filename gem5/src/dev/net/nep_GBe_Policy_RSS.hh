#ifndef __DEV_NET_NEPU_GBE_POLICY_RSS_HH__
#define __DEV_NET_NEPU_GBE_POLICY_RSS_HH__

#include "nep_GBe.hh"
#include "base/inet.hh"

class RxSidePolicyRSS : public RxSidePolicy {
  private:
    int idx_last_picked_entry=0;                  // Val for RoundRobin
    int calcHashValForEnQ(EthPacketPtr epkt);   // enQ Policy : hashing
    int tryRounRobinChoose(uint64_t map,int len);                   // send Policy : roundRobin
    static const int bucket_size = 128;
    int num_rx_queue;
    int indirection_table[bucket_size];
    void initIndirTable();

  public:
    virtual int getTargetRxQ(EthPacketPtr epkt);
    virtual int CurrentSender(uint64_t map, int len);
    virtual bool isAvailable(int idx);
    RxSidePolicyRSS(NepGbE* _igbe);
    RxSidePolicyRSS(NepGbE* _igbe, int num_rx_q);
    virtual ~RxSidePolicyRSS(){}

};

class TxSidePolicyRSS : public TxSidePolicy {
  private:
    int idx_last_picked_entry=0;                  // Val for RoundRobin
    int idx_last_wire=0;                  // Val for RoundRobin

  public:
    virtual int CurrentTx(char* specific);
    virtual int CurrentTxWire(char* specific);
    virtual bool isAvailable(int idx){return false;}
    TxSidePolicyRSS(NepGbE* __igbe);
    virtual ~TxSidePolicyRSS(){}

};

#endif // __DEV_NET_NEPU_GBE_POLICY_RSS_HH__