#include <common.h>
#include <net.h>
#include "recoveryProto.h"

static const unsigned int RECOVERY_PORT = 999;
static const char WELLCOME_STRING[] = "WELLCOME";

static int theRecoveryStatus = 0;

static void net_handler(uchar *pkt, unsigned dport,
			struct in_addr sip, unsigned sport, unsigned len)
{
    printf("Got packet %d %s\n",dport,pkt);

	if (dport != RECOVERY_PORT) {
		return;
	}

    static const char RECOVERY_MAGIC[] = "RECOVERY";
    static const char GO_MAGIC[] = "GO";
    if (memcmp(pkt,RECOVERY_MAGIC,ARRAY_SIZE(RECOVERY_MAGIC)-1)==0)
        theRecoveryStatus = 1;
    if (memcmp(pkt,GO_MAGIC,ARRAY_SIZE(GO_MAGIC)-1)==0)
        net_set_state(NETLOOP_SUCCESS);
}


void RecoveryProtoInit (void)
{
    theRecoveryStatus = 0;
    puts("Send wellcome \n");
	net_set_udp_handler(net_handler);


    uchar * pkt = (uchar *)(net_tx_packet + net_eth_hdr_size() + IP_UDP_HDR_SIZE);
    memcpy(pkt,WELLCOME_STRING,ARRAY_SIZE(WELLCOME_STRING)-1);
    struct in_addr	pboard_ip = string_to_ip("172.31.4.252");
    net_send_udp_packet(net_server_ethaddr,pboard_ip,  999, 999
        ,ARRAY_SIZE(WELLCOME_STRING)-1);
}

int     RecoveryProtoShouldRecovery()
{
    return theRecoveryStatus;
}


