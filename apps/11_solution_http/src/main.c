#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/http/client.h>

// Custom libraries
#include "wifi.h"

// WiFi settings
#define WIFI_SSID "MySSID"
#define WIFI_PSK "MyPassword"

// HTTP GET settings
#define HTTP_HOST "example.com"
#define HTTP_URL "/"
#define HTTP_TIMEOUT_MS 3000

// HTTP buffer settings
#define HTTP_RECV_BUF_LEN 512

// Globals
static uint8_t recv_buf[HTTP_RECV_BUF_LEN];

// HTTP request callback
static void response_callback(struct http_response *resp,
			                  enum http_final_call final_data,
			                  void *user_data)
{
    char temp_buf[HTTP_RECV_BUF_LEN + 1];

    // Check if we still have more data to receive
	if (final_data == HTTP_DATA_MORE) {
		printk("Partial data received (%d bytes)\r\n", resp->data_len);
	} else if (final_data == HTTP_DATA_FINAL) {
		printk("All data received (%d bytes)\r\n", resp->data_len);
	}

    // Print the received data (up to data_len bytes)
    memcpy(temp_buf, resp->recv_buf, resp->data_len);
    temp_buf[resp->data_len] = '\0';
    printk("Received data:\r\n%s\r\n", temp_buf);
}


// Print the results of a DNS lookup
void print_addrinfo(struct zsock_addrinfo **results)
{
	char ipv4[INET_ADDRSTRLEN];
	char ipv6[INET6_ADDRSTRLEN];
	struct sockaddr_in *sa;
	struct sockaddr_in6 *sa6;
	struct zsock_addrinfo *rp;
	
    // Iterate through the results
	for (rp = *results; rp != NULL; rp = rp->ai_next) {

        // Print the IP address (IPv4)
		if (rp->ai_addr->sa_family == AF_INET) {
			sa = (struct sockaddr_in *) rp->ai_addr;
			zsock_inet_ntop(AF_INET, &sa->sin_addr, ipv4, INET_ADDRSTRLEN);
			printf("IPv4: %s\n", ipv4);
		}

        // Print the IP address (IPv6)
		if (rp->ai_addr->sa_family == AF_INET6) {
			sa6 = (struct sockaddr_in6 *) rp->ai_addr;
			zsock_inet_ntop(AF_INET6, &sa6->sin6_addr, ipv6, INET6_ADDRSTRLEN);
			printf("IPv6: %s\n", ipv6);
		}
	}
}

int main(void)
{
    struct zsock_addrinfo hints;
    struct zsock_addrinfo *res;
    int sock;
    int ret;
    struct http_request req;

    printk("HTTP Client Subsystem Demo\r\n");

    // Initialize WiFi
    wifi_init();

    // Connect to the WiFi network (blocking)
    ret = wifi_connect(WIFI_SSID, WIFI_PSK);
    if (ret < 0) {
        printk("Error (%d): WiFi connection failed\r\n", ret);
        return 0;
    }

    // Wait to receive an IP address (blocking)
    wifi_wait_for_ip_addr();

    // Clear and set address info
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;          // IPv4
    hints.ai_socktype = SOCK_STREAM;    // TCP socket

    // Perform DNS lookup
    printk("Performing DNS lookup...\r\n");
    ret = zsock_getaddrinfo(HTTP_HOST, "80", &hints, &res);
    if (ret != 0) {
        printk("Error (%d): could not perform DNS lookup\r\n", ret);
        return 0;
    }

    // Print the results of the DNS lookup
    print_addrinfo(&res);

    // Create a new socket
    sock = zsock_socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) {
        printk("Error (%d): could not create socket\r\n", errno);
        return 0;
    }
    printk("Socket created. File descriptor: %d\r\n", sock);

    // Connect the socket
    ret = zsock_connect(sock, res->ai_addr, res->ai_addrlen);
    if (ret < 0) {
        printk("Error (%d): could not connect the socket\r\n", errno);
        return 0;
    }

    // Configure request
    memset(&req, 0, sizeof(req));
    req.method = HTTP_GET;
    req.url = HTTP_URL;
    req.host = HTTP_HOST;
    req.protocol = "HTTP/1.1";
    req.response = response_callback;
    req.recv_buf = recv_buf;
    req.recv_buf_len = sizeof(recv_buf);

    // Make request
    ret = http_client_req(sock, &req, HTTP_TIMEOUT_MS, NULL);
    if (ret < 0) {
        printk("Error (%d): HTTP request failed\r\n", ret);
        return 0;
    }

    // Close the socket
    zsock_close(sock);

    // Do nothing
    while (1) {
        k_sleep(K_FOREVER);
    }

	return 0;
}
