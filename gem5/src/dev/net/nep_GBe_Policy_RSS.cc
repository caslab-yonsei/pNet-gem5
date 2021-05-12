#include "nep_GBe_Policy_RSS.hh"
#include "base/inet.hh"

#include "debug/NepNicRxPolicyRss.hh"

// BST
int RxSidePolicyRSS::calcHashValForEnQ(EthPacketPtr ethpkt)
{
    uint32_t rss_src_ip;   //rss_source_ip
    uint32_t rss_dst_ip;   //rss_destination_ip
    uint16_t rss_src_port; //rss_soruce_port
    uint16_t rss_dst_port; //rss_destination_port

    DPRINTF(NepNicRxPolicyRss, "RxSidePolicyRSS::calcHashValForEnQ called.\n");
    //Net::IpPtr(ethpkt);
    Net::IpPtr ip(ethpkt);      //ip4정보
    DPRINTF(NepNicRxPolicyRss, "RxSidePolicyRSS::calcHashValForEnQ get ipPtr %#llx\n", ip);

    // If not IP. Send to Queue 0 (as default. I recommand adding an option to send other queue)
    if(!ip) 
    {
        // What? It is not an Internet Protocol Packet!!!
        // Who are you?
        // I am Fine!
        // Ok. Mr.Fine.. Ms.Fine?
        // Ehwu...

        DPRINTF(NepNicRxPolicyRss, "RxSidePolicyRSS::calcHashValForEnQ received non-IP packet!\n");
        return 0;
    }

    //Ip6Ptr ip6(ePtr);//ip6정보 일단 고려 안함
    rss_dst_ip = ip->dst();
    DPRINTF(NepNicRxPolicyRss, "RxSidePolicyRSS::dst_ip %d\n",rss_dst_ip);
    rss_src_ip = ip->src();
    DPRINTF(NepNicRxPolicyRss, "RxSidePolicyRSS::src_ip %d\n",rss_src_ip);
    Net::TcpPtr tcp = Net::TcpPtr(ip); //tcp포인터 가져오기 null이면 udp
    Net::UdpPtr udp = Net::UdpPtr(ip);
    if (tcp)
    {
        rss_src_port = tcp->sport();
        rss_dst_port = tcp->dport();
    }
    else if (udp)
    {
        //Net::UdpPtr udp = Net::UdpPtr(ip); //udp포인터 가져오기 null이면 tcp
        rss_src_port = udp->sport();
        rss_dst_port = udp->dport();
    }
    else{
        rss_src_port = 0;
        rss_dst_port = 0;
        return 0;
    }
    DPRINTF(NepNicRxPolicyRss, "RxSidePolicyRSS::src_port %d\n",rss_src_port);
    DPRINTF(NepNicRxPolicyRss, "RxSidePolicyRSS::dst_port %d\n",rss_dst_port);
    uint32_t rss_port = (rss_src_port << 16) + rss_dst_port;
    //assemble input
    uint32_t input[3];
    input[0] = rss_src_ip;
    input[1] = rss_dst_ip;
    input[2] = rss_port;

    //set seed
    uint32_t seed[6];
    seed[0] = 0x3a42624c;
    seed[1] = 0x41a926fc;
    seed[2] = 0x5adf32f8;
    seed[3] = 0x2944c283;
    seed[4] = 0x9ab381ab;
    seed[5] = 0x428910a3;
    uint32_t key;
    uint32_t result = 0;
    uint32_t assist_key;

    //calculrate hash
    for (int i = 0; i < 3; i++)
    {
        key = seed[i];
        //assist_key use to assist left shift of seed array
        assist_key = seed[i + 1];
        for (uint32_t mask = 1; mask != 0; mask = mask << 1)
        {
            if ((mask & input[i]) != 0)
                result ^= key;
            key = ((key << 1) | ((assist_key & (1 << 31)) >> 31));
            assist_key = assist_key << 1;
        }
    }
    //get Queue num using table
    //uint32_t Queue_num = result % getNepGbE()->gem5NumRunningContexts();
    DPRINTF(NepNicRxPolicyRss, "RSS Hash. result is %d\r\n", result);
    
    // TEMP
    uint32_t bucket = result % bucket_size;
    //return rss_dst_port % 4;	    
    uint32_t queue_num = indirection_table[bucket];
    //return result;	
     //return Queue_num;	    //return Queue_num;
    return queue_num;
}

// BST need to be edit!. tick은 켜졌으나 실제로는 pkt이 없을 경우 스킵할 수 있도록.
int RxSidePolicyRSS::tryRounRobinChoose(uint64_t map, int len)
{
    int candidate_idx=(idx_last_picked_entry+1) % len;
    while(candidate_idx!=idx_last_picked_entry){
        if((1<<candidate_idx) & map){
            idx_last_picked_entry=candidate_idx;
            return candidate_idx;
        }
        candidate_idx++;
        if(candidate_idx>=len){
            candidate_idx=0;
        }
    }
    if((1<<idx_last_picked_entry)&map){
        return idx_last_picked_entry;    
    }
    return -1;
}

// BST 일단 넘겨도 됨
bool RxSidePolicyRSS::isAvailable(int idx)
{
    return 0;
}

RxSidePolicyRSS::RxSidePolicyRSS(NepGbE *_igbe)
    : RxSidePolicy(_igbe)
{
    idx_last_picked_entry = 0;
    initIndirTable();
}

 RxSidePolicyRSS::RxSidePolicyRSS(NepGbE *_igbe,int num_rx_q)
    : RxSidePolicy(_igbe)
{
    idx_last_picked_entry = 0;
    num_rx_queue = num_rx_q;
    initIndirTable();
}

int 
RxSidePolicyRSS::getTargetRxQ(EthPacketPtr epkt)
{
    DPRINTF(NepNicRxPolicyRss, "RxSidePolicyRSS::getTargetRxQ Called!\n");
    // SHIN TEST
    // temp sime hash
    /*
    uint64_t *data = (uint64_t*)(epkt->data);
    int q = (*data) % 4;
    DPRINTF(NepNicRxPolicyRss, "RSS Hash. %s to %d to %d\r\n", (char*)data, *data, q);
    return q;
    */
    // TEMP
    //return 1;
    return calcHashValForEnQ(epkt);
    //return 0;
}

int 
RxSidePolicyRSS::CurrentSender(uint64_t map, int len)
{
    //DPRINTF(NepNicRxPolicyRss, "RxSidePolicyRSS::CurrentSender Called!\n");
    return tryRounRobinChoose(map,len);
}

TxSidePolicyRSS::TxSidePolicyRSS(NepGbE* __igbe)
    :TxSidePolicy(__igbe)
{
    idx_last_picked_entry = 0;
}

int
TxSidePolicyRSS::CurrentTx(char* specific)
{
    int num_tx_queue = *(int*)specific;
    idx_last_picked_entry++;
    idx_last_picked_entry %= num_tx_queue;
    return idx_last_picked_entry;
    //return 0;
}

int
TxSidePolicyRSS::CurrentTxWire(char* specific)
{
    int num_tx_queue = *(int*)specific;
    idx_last_wire++;
    idx_last_wire %= num_tx_queue;
    return idx_last_wire;
    //return 0;
}

void RxSidePolicyRSS::initIndirTable()
{
    //int total_core_num = 4;
    //DPRINTF(NepNicRxManager, "total core num: %d\n", total_core_num);
    for(int i=0; i < bucket_size; i++)
    {
        indirection_table[i] = i % num_rx_queue;
    }
    DPRINTF(NepNicRxManager, "indirection table init completed");
} 	