/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Ali Saidi
 */

/* @file
 * Device model for Intel's 8254x line of gigabit ethernet controllers.
 */

#ifndef __DEV_NET_NEPGBE_HH__
#define __DEV_NET_NEPGBE_HH__

#include <deque>
#include <string>

#include "base/inet.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/EthernetDesc.hh"
#include "debug/EthernetIntr.hh"
#include "dev/net/etherdevice.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherpkt.hh"
#include "dev/net/pktfifo.hh"
#include "dev/pci/device.hh"
#include "sim/eventq.hh"
#include "sim/serialize.hh"

// SHIN. Loadgen
#include "params/NepGbE.hh"
#include "debug/NepNicRxManager.hh"
#include "dev/net/nep_GBe_defs.hh"

// For MSI/INT controll
#define INT_LEGACY 0
#define INT_MSI 1
#define INT_MSIX 2

class NepGbE;
class NepGbEInt;
class RxSidePolicy;
class TxSidePolicy;
class MultiDmaEngine;


class NepGbE : public EtherDevice
{
  private:
    NepGbEInt *etherInt;
    

    //std::vector<DmaEngine*> dma_engines;

    // device registers
    iGbReg::Regs regs;

    // eeprom data, status and control bits
    int eeOpBits, eeAddrBits, eeDataBits;
    uint8_t eeOpcode, eeAddr;
    uint16_t flash[iGbReg::EEPROM_SIZE];

    // packet fifos
    //PacketFifo rxFifo;
    PacketFifo txFifo;

    // Packet that we are currently putting into the txFifo
    EthPacketPtr txPacket;

    // Should to Rx/Tx State machine tick?
    bool inTick;
    //bool rxTick;
    bool txTick;
    bool txFifoTick;

    bool rxDmaPacket;

    int num_of_queues;

    // SHIN. Test
    int test_count = 0;
    EthPacketPtr test_packet;

    // Number of bytes copied from current RX packet
    unsigned pktOffset;

    // Delays in managaging descriptors
    Tick fetchDelay, wbDelay;
    Tick fetchCompDelay, wbCompDelay;
    Tick rxWriteDelay, txReadDelay;

    // SHIN. Load Generator
    friend class LoadGenerator;
    friend class MultiDmaEngine;


    // Event and function to deal with RDTR(인터럽트) timer expiring 


    void rdtrProcess() {
        rxDescCache.writeback(0);
        DPRINTF(EthernetIntr,
                "Posting RXT interrupt because RDTR timer expired\n");
        postInterrupt(iGbReg::IT_RXT);
    }

    EventFunctionWrapper rdtrEvent;

    // Event and function to deal with RADV(절대시간)) timer expiring
    void radvProcess() {
        rxDescCache.writeback(0);
        DPRINTF(EthernetIntr,
                "Posting RXT interrupt because RADV timer expired\n");
        postInterrupt(iGbReg::IT_RXT);
    }

    EventFunctionWrapper radvEvent;

    // Event and function to deal with TADV timer expiring
    void tadvProcess() {
        txDescCache.writeback(0);
        DPRINTF(EthernetIntr,
                "Posting TXDW interrupt because TADV timer expired\n");
        postInterrupt(iGbReg::IT_TXDW);
    }

    EventFunctionWrapper tadvEvent;

    // Event and function to deal with TIDV timer expiring
    void tidvProcess() {
        txDescCache.writeback(0);
        DPRINTF(EthernetIntr,
                "Posting TXDW interrupt because TIDV timer expired\n");
        postInterrupt(iGbReg::IT_TXDW);
    }
    EventFunctionWrapper tidvEvent;

    // Main event to tick the device
    void tick();
    EventFunctionWrapper tickEvent;

    // SHIN BST
    void SimpleLoad();
    EventFunctionWrapper loadGenEvent;
    int addrToQid(Addr daddr);

    uint64_t macAddr;

    //void rxStateMachine();
    void txStateMachine();
    int txWire();

    /** Write an interrupt into the interrupt pending register and check mask
     * and interrupt limit timer before sending interrupt to CPU
     * @param t the type of interrupt we are posting
     * @param now should we ignore the interrupt limiting timer
     */
    void postInterrupt(iGbReg::IntTypes t, bool now = false);
    void postMqInterrupt(iGbReg::IntTypes t, int queue = -1, bool now = false);

    // For MSI
    void postInterrupt_MSI(iGbReg::IntTypes t, int queue = -1, bool now = false);

    // Legacy interrupts
    void postInterrupt_LEGACY(iGbReg::IntTypes t, bool now = false);

    /** Check and see if changes to the mask register have caused an interrupt
     * to need to be sent or perhaps removed an interrupt cause.
     */
    void chkInterrupt();

    // SHIN. MSI version of chkInterrupt
    void chkMSI();

    /** Send an interrupt to the cpu
     */
    void delayIntEvent();
    void cpuPostInt();
    
    // Event to moderate interrupts
    EventFunctionWrapper interEvent;
    //EventFunctionWrapper msiEvent;

    /** Clear the interupt line to the cpu
     */
    void cpuClearInt();

    Tick intClock() { return SimClock::Int::ns * 1024; }

    /** This function is used to restart the clock so it can handle things like
     * draining and resume in one place. */
    void restartClock();

    /** Check if all the draining things that need to occur have occured and
     * handle the drain event if so.
     */
    void checkDrain();

    


    template<class T>
    class DescCache : public Serializable
    {
      protected:
        virtual Addr descBase() const = 0;
        virtual long descHead() const = 0;
        virtual long descTail() const = 0;
        virtual long descLen() const = 0;
        virtual void updateHead(long h) = 0;
        virtual void enableSm() = 0;
        virtual void actionAfterWb() {}
        virtual void fetchAfterWb() = 0;

        typedef std::deque<T *> CacheType;
        CacheType usedCache;
        CacheType unusedCache;

        T *fetchBuf;
        T *wbBuf;

        // Pointer to the device we cache for
        NepGbE *igbe;

        // Name of this  descriptor cache
        std::string _name;

        // How far we've cached
        int cachePnt;

        // The size of the descriptor cache
        int size;

        // How many descriptors we are currently fetching
        int curFetching;

        // How many descriptors we are currently writing back
        int wbOut;

        // if the we wrote back to the end of the descriptor ring and are going
        // to have to wrap and write more
        bool moreToWb;

        // What the alignment is of the next descriptor writeback
        Addr wbAlignment;

        /** The packet that is currently being dmad to memory if any */
        EthPacketPtr pktPtr;

        /** Shortcut for DMA address translation */
        Addr pciToDma(Addr a) { return igbe->pciToDma(a); }

        int idx = 0;
        int offset = 0;   // 대충 크기를 전체 수로 나눠서 뭐 하는 걸로 하자.

      public:
        void setIdx(int idx_){idx=idx_;}

        // SHIN. Set offsets for multiQ support!
        void setOffset(Addr new_offset) {offset = new_offset;}

        /** Annotate sm*/
        std::string annSmFetch, annSmWb, annUnusedDescQ, annUsedCacheQ,
            annUsedDescQ, annUnusedCacheQ, annDescQ;

        DescCache(NepGbE *i, const std::string n, int s);
        virtual ~DescCache();

        std::string name() { return _name; }

        /** If the address/len/head change when we've got descriptors that are
         * dirty that is very bad. This function checks that we don't and if we
         * do panics.
         */
        void areaChanged();

        void writeback(Addr aMask);
        void writeback1();
        EventFunctionWrapper wbDelayEvent;

        /** Fetch a chunk of descriptors into the descriptor cache.
         * Calls fetchComplete when the memory system returns the data
         */
        void fetchDescriptors();
        void fetchDescriptors1();
        EventFunctionWrapper fetchDelayEvent;

        /** Called by event when dma to read descriptors is completed
         */
        void fetchComplete();
        EventFunctionWrapper fetchEvent;

        /** Called by event when dma to writeback descriptors is completed
         */
        void wbComplete();
        EventFunctionWrapper wbEvent;

        /* Return the number of descriptors left in the ring, so the device has
         * a way to figure out if it needs to interrupt.
         */
        unsigned
        descLeft() const
        {
            unsigned left = unusedCache.size();
            if (cachePnt > descTail())
                left += (descLen() - cachePnt + descTail());
            else
                left += (descTail() - cachePnt);

            return left;
        }

        /* Return the number of descriptors used and not written back.
         */
        unsigned descUsed() const { return usedCache.size(); }

        /* Return the number of cache unused descriptors we have. */
        unsigned descUnused() const { return unusedCache.size(); }

        /* Get into a state where the descriptor address/head/etc colud be
         * changed */
        void reset();


        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

        virtual bool hasOutstandingEvents() {
            return wbEvent.scheduled() || fetchEvent.scheduled();
        }

    };


    class RxDescCache : public DescCache<iGbReg::RxDesc>
    {
    //   // SHIN. For multi Q
    //   private:
    //     int idx = 0;
    //     int offset = 0;   // 대충 크기를 전체 수로 나눠서 뭐 하는 걸로 하자.

      protected:
        Addr descBase() const override { 
            //DPRINTF(NepNicRxManager, "Desc idx %d\n", idx);
            return igbe->regs.nep_ex_regs[idx].rdba(); 
        }
        long descHead() const override { return igbe->regs.nep_ex_regs[idx].rdh(); }
        long descLen() const override { return igbe->regs.nep_ex_regs[idx].rdlen() >> 4; }
        long descTail() const override { return igbe->regs.nep_ex_regs[idx].rdt(); }
        void updateHead(long h) override { igbe->regs.nep_ex_regs[idx].rdh(h); }
        void enableSm() override;
        void fetchAfterWb() override {
            if (!igbe->mqManager.checkIdxRxTick(idx) && igbe->drainState() == DrainState::Running)
                fetchDescriptors();
        }

        bool pktDone;

        /** Variable to head with header/data completion events */
        int splitCount;

        /** Bytes of packet that have been copied, so we know when to
            set EOP */
        unsigned bytesCopied;

      public:
        RxDescCache(NepGbE *i, std::string n, int s);
        // void setIdx(int idx_){idx=idx_;}

        // // SHIN. Set offsets for multiQ support!
        // void setOffset(Addr new_offset) {offset = new_offset;}

        /** Write the given packet into the buffer(s) pointed to by the
         * descriptor and update the book keeping. Should only be called when
         * there are no dma's pending.
         * @param packet ethernet packet to write
         * @param pkt_offset bytes already copied from the packet to memory
         * @return pkt_offset + number of bytes copied during this call
         */
        int writePacket(EthPacketPtr packet, int pkt_offset);

        /** Called by event when dma to write packet is completed
         */
        void pktComplete();

        /** Check if the dma on the packet has completed and RX state machine
         * can continue
         */
        bool packetDone();

        EventFunctionWrapper pktEvent;

        // Event to handle issuing header and data write at the same time
        // and only callking pktComplete() when both are completed
        void pktSplitDone();
        EventFunctionWrapper pktHdrEvent;
        EventFunctionWrapper pktDataEvent;

        bool hasOutstandingEvents() override;

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };
    friend class RxDescCache;

    RxDescCache rxDescCache;

    class TxDescCache  : public DescCache<iGbReg::TxDesc>
    {
      protected:
        Addr descBase() const override { return igbe->regs.nep_ex_regs[idx].tdba(); }
        long descHead() const override { return igbe->regs.nep_ex_regs[idx].tdh(); }
        long descTail() const override { return igbe->regs.nep_ex_regs[idx].tdt(); }
        long descLen() const override { return igbe->regs.nep_ex_regs[idx].tdlen() >> 4; }
        void updateHead(long h) override { igbe->regs.nep_ex_regs[idx].tdh(h); }
        void enableSm() override;
        void actionAfterWb() override;
        void fetchAfterWb() override {
            if (!igbe->mqManager.checkIdxTxTick(idx) && igbe->drainState() == DrainState::Running)
                fetchDescriptors();
        }



        bool pktDone;
        bool isTcp;
        bool pktWaiting;
        bool pktMultiDesc;
        Addr completionAddress;
        bool completionEnabled;
        uint32_t descEnd;


        // tso variables
        bool useTso;
        Addr tsoHeaderLen;
        Addr tsoMss;
        Addr tsoTotalLen;
        Addr tsoUsedLen;
        Addr tsoPrevSeq;
        Addr tsoPktPayloadBytes;
        bool tsoLoadedHeader;
        bool tsoPktHasHeader;
        uint8_t tsoHeader[256];
        Addr tsoDescBytesUsed;
        Addr tsoCopyBytes;
        int tsoPkts;

      public:
        TxDescCache(NepGbE *i, std::string n, int s);

        /** Tell the cache to DMA a packet from main memory into its buffer and
         * return the size the of the packet to reserve space in tx fifo.
         * @return size of the packet
         */
        unsigned getPacketSize(EthPacketPtr p);
        void getPacketData(EthPacketPtr p);
        void processContextDesc();

        /** Return the number of dsecriptors in a cache block for threshold
         * operations.
         */
        unsigned
        descInBlock(unsigned num_desc)
        {
            return num_desc / igbe->cacheBlockSize() / sizeof(iGbReg::TxDesc);
        }

        /** Ask if the packet has been transfered so the state machine can give
         * it to the fifo.
         * @return packet available in descriptor cache
         */
        bool packetAvailable();

        /** Ask if we are still waiting for the packet to be transfered.
         * @return packet still in transit.
         */
        bool packetWaiting() { return pktWaiting; }

        /** Ask if this packet is composed of multiple descriptors
         * so even if we've got data, we need to wait for more before
         * we can send it out.
         * @return packet can't be sent out because it's a multi-descriptor
         * packet
         */
        bool packetMultiDesc() { return pktMultiDesc;}

        /** Called by event when dma to write packet is completed
         */
        void pktComplete();
        EventFunctionWrapper pktEvent;

        void headerComplete();
        EventFunctionWrapper headerEvent;


        void completionWriteback(Addr a, bool enabled) {
            DPRINTF(EthernetDesc,
                    "Completion writeback Addr: %#x enabled: %d\n",
                    a, enabled);
            completionAddress = a;
            completionEnabled = enabled;
        }

        bool hasOutstandingEvents() override;

        void nullCallback() {
            DPRINTF(EthernetDesc, "Completion writeback complete\n");
        }
        EventFunctionWrapper nullEvent;

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };

    friend class TxDescCache;

    TxDescCache txDescCache;

    // struct RxComponentSet {
    //     RxDescCache rxDescCache;
    //     PacketFifo rxfifo;
    //     bool available = true;    // rxTick
    //     bool rxDmaPacket = false;
    //     int idxComponent;
    //     int pktOffset = 0;
    //     NepGbE* igbe;
    //     std::string _name;
    //     RxComponentSet(NepGbE* igbe, int rxfifo_size, int desc_size, std::string _manager_name, int idx);
    //     std::string name(){return _name;}

    //     // Intr...
    //     EventFunctionWrapper interMultEvent;
    //     EventFunctionWrapper rdtrMultEvent;
    //     EventFunctionWrapper radvMultEvent;
    //     EventFunctionWrapper tadvMultEvent;
    //     EventFunctionWrapper tidvMultEvent;
        
    //     void postInterruptMultIntr(iGbReg::IntTypes t, bool now = false);
    //     void cpuPostMultInt();
    //     void cpuClearMultiInt();
    //     void delayIntEventMultIntr();
    //     void rdtrProcessMultIntr();
    //     void radvProcessMultIntr();
    //     void tadvProcessMultIntr();
    //     void tidvProcessMultIntr();

    //     void chkMultiMSI();
    // };

    // SHIN. Structure that collects variables that need
    // individual management in RSS-enabled device
    struct MultiQueueComponentSet : public Serializable {
        RxDescCache rxDescCache;
        PacketFifo rxfifo;

        TxDescCache txDescCache;
        PacketFifo txfifo;

        bool rx_available = false;    // rxTick
        bool tx_available = false;    // txTick
        bool txfifo_available = true;    // txFifoTick
        bool rxDmaPacket = false;
        EthPacketPtr txPacket;
        
        int idxComponent;
        int pktOffset = 0;
        NepGbE* igbe;
        std::string _name;
        MultiQueueComponentSet(NepGbE* igbe, int rxfifo_size, int rx_desc_size, 
            int txfifo_size, int tx_desc_size, std::string _manager_name, int idx);
        std::string name(){return _name;}

        // Intr...
        EventFunctionWrapper interMultEvent;
        EventFunctionWrapper rdtrMultEvent;
        EventFunctionWrapper radvMultEvent;
        EventFunctionWrapper tadvMultEvent;
        EventFunctionWrapper tidvMultEvent;

        Tick lastMultInterrupt = 0;
        
        void postInterruptMultIntr(iGbReg::IntTypes t, bool now = false);
        void cpuPostMultInt();
        void cpuClearMultiInt();
        void delayIntEventMultIntr();
        void rdtrProcessMultIntr();
        void radvProcessMultIntr();
        void tadvProcessMultIntr();
        void tidvProcessMultIntr();

        void chkMultiMSI();

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };

    // struct TxComponentSet {
    //     TxDescCache txDescCache;
    //     PacketFifo txfifo;
    //     bool available = true;    // rxTick
    //     bool txDmaPacket = false;
    //     int idxComponent;
    //     int pktOffset = 0;

    //     NepGbE* igbe;
    //     std::string _name;

    //     TxComponentSet(NepGbE* igbe, int rxfifo_size, int desc_size, std::string _manager_name, int idx);
    //     std::string name(){return _name;}
    //     // Intr...
    //     EventFunctionWrapper interMultEvent;
    //     EventFunctionWrapper rdtrMultEvent;
    //     EventFunctionWrapper radvMultEvent;
    //     EventFunctionWrapper tadvMultEvent;
    //     EventFunctionWrapper tidvMultEvent;
    // };

    // SHIN. As a manager of Rx, it is possible to change and apply policies.
    class MultiQueueManager : public Serializable {
      private:
        // Managed Component Vectors
        std::vector<MultiQueueComponentSet*> mq_component_sets;
        //std::vector<TxComponentSet*> tx_component_sets; Not Implemented yet...

        // policy
        RxSidePolicy* rx_policy;
        TxSidePolicy* tx_policy;
        // Pointer to NepGbE
        NepGbE *igbe;
        // common vals
        std::string _name;
        // Active queue
        // Do not use
        uint64_t map_active_queues = -1;
        int num_rx_queues = 1;
        int num_tx_queues = 1;

        // To handle errors
        int idx_last_enQ_try=0;
        int idx_last_deQ_try=0;

      // Functions
      private:
        // Data return functions to ease implementation
        // Wire to NIC
        MultiQueueComponentSet& getTargetRxQ(EthPacketPtr epkt);
        
        bool isQueueIdxOn(int idx){return (1<<idx) & map_active_queues;}
        int CurrentRx();
        int CurrentTx();
        int CurrentTxWire();

      public:
        std::string name() { return _name; }
        MultiQueueComponentSet& getCompByIdx(int idx){return *mq_component_sets[idx];}
        
        
        
        // To DO
        bool InsertEthPktToRxQ(EthPacketPtr epkt);
        bool areaChanged(); // Clear All Desc
        void rxIdxAreaChanged(int idx){mq_component_sets[idx]->rxDescCache.areaChanged();};
        void txIdxAreaChanged(int idx){mq_component_sets[idx]->txDescCache.areaChanged();};
        void setRxDecsAddr();
        void setTxDecsAddr();
        void setIdxRxTick(int idx, bool val);//set i-th component tick to value
        void setIdxTxTick(int idx, bool val);//set i-th component tick to value
        void setIdxTxFifoTick(int idx, bool val);//set i-th component tick to value
        void allRXTickOn();//set all component on
        void allTXTickOn();//set all component on
        void allRXTickOff();//set all compnent off
        void allTXTickOff();//set all compnent off
        void allTxFifoTickOn();
        void allTxFifoTickOff();
        // FW
        int RunRxTick();   // Bring NepGbE::rxStateMachine() from NepGbE
        int RunTxTick();   // Bring NepGbE::rxStateMachine() from NepGbE
        bool checkIdxRxTick(int idx){return mq_component_sets[idx]->rx_available;}//check i-th component RxTick
        bool checkIdxTxTick(int idx){return mq_component_sets[idx]->tx_available;}//check i-th component RxTick
        bool checkIdxTxFifoTick(int idx){return mq_component_sets[idx]->txfifo_available;}//check i-th component RxTick
        bool checkRxTick(){
            bool rxtick = false;
            for(int i = 0; i < num_rx_queues; i++)
            {
                rxtick |= mq_component_sets[i]->rx_available;
            }
            return rxtick;
        }
        bool checkTxTick(){
            bool txtick = false;
            for(int i = 0; i < num_tx_queues; i++)
            {
                txtick |= mq_component_sets[i]->tx_available;
            }
            return txtick;
        }
        bool checkTxFifoTick(){
            bool txtick = false;
            for(int i = 0; i < num_tx_queues; i++)
            {
                txtick |= mq_component_sets[i]->txfifo_available;
            }
            return txtick;
        }

        bool hasOutstandingEvents(){
            bool has = false;
            for(int i = 0; i < num_rx_queues; i++)
            {
                has |= mq_component_sets[i]->rxDescCache.hasOutstandingEvents();
            }
            return has;
        }

        bool hasOutstandingEventsTx(){
            bool has = false;
            for(int i = 0; i < num_tx_queues; i++)
            {
                has |= mq_component_sets[i]->txDescCache.hasOutstandingEvents();
            }
            return has;
        }

        void chkMultiMSIs();


        // I made it, but I don't recommend it.
        //RxDescCache& getRxDescCache(int idx);
        //PacketFifo& getPacketFifo(int idx);
        //bool isAvailable(int idx);
        //bool get_rxDmaPacket(int idx);
        
        void rxDescCache_Wb(int idx){mq_component_sets[idx]->rxDescCache.writeback(0);}
        void rxDescCache_reset(int idx){mq_component_sets[idx]->rxDescCache.reset();}
        void rxDescCache_fetch(int idx){mq_component_sets[idx]->rxDescCache.fetchDescriptors();}
        
        void txDescCache_completionWriteback(int idx, Addr a, bool enabled)
            {mq_component_sets[idx]->txDescCache.completionWriteback(a, enabled);}
        void txDescCache_reset(int idx){mq_component_sets[idx]->txDescCache.reset();}
        void txDescCache_fetch(int idx){mq_component_sets[idx]->txDescCache.fetchDescriptors();}

        int getNumTxQueues(){return num_tx_queues;}

        PacketFifo& getTxFifo();
        
        MultiQueueManager(NepGbE* __igbe, int num_rx_q, int rx_queue_max_size, int rx_desc_max_size,
                                          int num_tx_q, int tx_queue_max_size, int tx_desc_max_size);
        ~MultiQueueManager();

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };
    MultiQueueManager mqManager;

  public:
    PARAMS(NepGbE);
    Tick global_int_time = 0;
    NepGbE(const Params &params);
    ~NepGbE();
    void init() override;

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    int getNumOfQueue(){return num_of_queues;}

    Tick lastInterrupt;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    Tick writeConfig(PacketPtr pkt) override;

    bool ethRxPkt(EthPacketPtr packet);
    void ethTxDone();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    DrainState drain() override;
    void drainResume() override;
    // int gem5NumRunningContexts()//BST get runningcontexts sys
    // {
    //   return sys->numRunningContexts();
    // }
  public:
    bool port_specific;
    int dist_rank;
};
class NepGbEInt : public EtherInt
{
  private:
    NepGbE *dev;
    int int_mode;

  public:
    NepGbEInt(const std::string &name, NepGbE *d)
        : EtherInt(name), dev(d), int_mode(INT_LEGACY)
    { }

    virtual bool recvPacket(EthPacketPtr pkt) { return dev->ethRxPkt(pkt); }
    virtual void sendDone() { dev->ethTxDone(); }
    int set_int_mode(int type) 
    {
        if(type < 0 || type > 2){
            return -1;}
        int_mode = type;
        return 0;
    }
    int get_int_mode() {return int_mode;}
};

class RxSidePolicy{
  protected:
    NepGbE* igbe;
  public:
    virtual int getTargetRxQ(EthPacketPtr epkt) {return 0;}
    virtual int CurrentSender(uint64_t map,int len){return 0;}
    virtual bool isAvailable(int idx){return false;}
    NepGbE* getNepGbE(){return igbe;}
    RxSidePolicy(NepGbE* __igbe);
    virtual ~RxSidePolicy(){}

};

class TxSidePolicy{
  private:
    NepGbE* igbe;
  public:
    virtual int CurrentTx(char* specific){return 0;}
    virtual int CurrentTxWire(char* specific){return 0;}
    virtual bool isAvailable(int idx){return false;}
    NepGbE* getNepGbE(){return igbe;}
    TxSidePolicy(NepGbE* __igbe);
    virtual ~TxSidePolicy(){}

};



#endif //__DEV_NET_I8254XGBE_HH__
