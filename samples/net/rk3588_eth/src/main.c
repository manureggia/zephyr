#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_ip.h>

LOG_MODULE_REGISTER(rk3588_eth, LOG_LEVEL_INF);

static K_SEM_DEFINE(dhcp_bound, 0, 1);
static struct net_mgmt_event_callback mgmt_cb;

static void ipv4_addr_cb(uint32_t mgmt_event, struct net_if *iface)
{
	if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
		return;
	}

	if (k_sem_count_get(&dhcp_bound) == 0U) {
		k_sem_give(&dhcp_bound);
	}
}

static void print_ipv4_address(struct net_if *iface)
{
	const struct net_if_ipv4 *ipv4 = net_if_ipv4_get(iface);
	char buf[NET_IPV4_ADDR_LEN] = {0};

	if (!ipv4 || !ipv4->unicast[0].is_used) {
		LOG_WRN("IPv4 address missing");
		return;
	}

	LOG_INF("got IPv4 %s",
		net_addr_ntop(AF_INET,
			      &ipv4->unicast[0].address.in_addr,
			      buf, sizeof(buf)));
}

int main(void)
{
	struct net_if *iface = net_if_get_default();
	int ret;

	if (iface == NULL) {
		LOG_ERR("No default network interface");
		return 0;
	}

	net_mgmt_init_event_callback(&mgmt_cb, ipv4_addr_cb,
				     NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&mgmt_cb);

	LOG_INF("waiting for IPv4 lease…");

	ret = k_sem_take(&dhcp_bound, K_SECONDS(30));
	if (ret == 0) {
		print_ipv4_address(iface);
	} else {
		LOG_WRN("Timed out while waiting for DHCP (ret %d)", ret);
	}

	for (;;) {
		k_sleep(K_SECONDS(5));
	}

	return 0;
}
