#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>

// Custom libraries
#include "wifi.h"

// WiFi settings
#define WIFI_SSID "MySSID"
#define WIFI_PSK "MyPassword"

// HTTP GET settings
#define HTTP_HOST "example.com"
#define HTTP_URL "/"

// Globals
static char response[512];

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

        // Print IPv4 address
        if (rp->ai_addr->sa_family == AF_INET) {
            sa = (struct sockaddr_in *)rp->ai_addr;
            zsock_inet_ntop(AF_INET, &sa->sin_addr, ipv4, INET_ADDRSTRLEN);
            printk("IPv4: %s\r\n", ipv4);
        }

        // Print IPv6 address
        if (rp->ai_addr->sa_family == AF_INET6) {
            sa6 = (struct sockaddr_in6 *)rp->ai_addr;
            zsock_inet_ntop(AF_INET6, &sa6->sin6_addr, ipv6, INET6_ADDRSTRLEN);
            printk("IPv6: %s\r\n", ipv6);
        }
    }
}

int main(void)
{
    struct zsock_addrinfo hints;
    struct zsock_addrinfo *res;
    char http_request[512];
    int sock;
    int len;
    uint32_t rx_total;
    int ret;

    printk("HTTP GET Demo\r\n");

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

    // Construct HTTP GET request
    snprintf(http_request,
             sizeof(http_request),
             "GET %s HTTP/1.1\r\nHost: %s\r\n\r\n",
             HTTP_URL,
             HTTP_HOST);

    // Clear and set address info
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;              // IPv4
    hints.ai_socktype = SOCK_STREAM;        // TCP socket

    // Perform DNS lookup
    printk("Performing DNS lookup...\r\n");
    ret = zsock_getaddrinfo(HTTP_HOST, "80", &hints, &res);
    if (ret != 0) {
        printk("Error (%d): Could not perform DNS lookup\r\n", ret);
        return 0;
    }

    // Print the results of the DNS lookup
    print_addrinfo(&res);

    // Create a new socket
    sock = zsock_socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) {
        printk("Error (%d): Could not create socket\r\n", errno);
        return 0;
    }

    // Connect the socket
    ret = zsock_connect(sock, res->ai_addr, res->ai_addrlen);
    if (ret < 0) {
        printk("Error (%d): Could not connect the socket\r\n", errno);
        return 0;
    }

    // Set the request
    printk("Sending HTTP request...\r\n");
    ret = zsock_send(sock, http_request, strlen(http_request), 0);
    if (ret < 0) {
        printk("Error (%d): Could not send request\r\n", errno);
        return 0;
    }

    // Print the response
    printk("Response:\r\n\r\n");
    rx_total = 0;
    while (1) {

        // Receive data from the socket
        len = zsock_recv(sock, response, sizeof(response) - 1, 0);

        // Check for errors
        if (len < 0) {
            printk("Receive error (%d): %s\r\n", errno, strerror(errno));
            return 0;
        }

        // Check for end of data
        if (len == 0) {
            break;
        }

        // Null-terminate the response string and print it
        response[len] = '\0';
        printk("%s", response);
        rx_total += len;
    }

    // Print the total number of bytes received
    printk("\r\nTotal bytes received: %u\r\n", rx_total);

    // Close the socket
    zsock_close(sock);

    return 0;
}
