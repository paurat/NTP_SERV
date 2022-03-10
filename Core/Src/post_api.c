#include "post_api.h"
#define USER_PASS_BUFSIZE 512

static void *current_connection;
static void *valid_connection;
static char last_user[USER_PASS_BUFSIZE];

err_t
httpd_post_begin(void *connection, const char *uri, const char *http_request, u16_t http_request_len, int content_len, char *response_uri, u16_t response_uri_len, u8_t *post_auto_wnd)
{
	LWIP_UNUSED_ARG(connection);
	LWIP_UNUSED_ARG(http_request);
	LWIP_UNUSED_ARG(http_request_len);
	LWIP_UNUSED_ARG(content_len);
	LWIP_UNUSED_ARG(post_auto_wnd);
	if (!memcmp(uri, "/info.html", 11)) {
		if (current_connection != connection) {
			current_connection = connection;
			valid_connection = NULL;
			/* default page is "login failed" */
			snprintf(response_uri, response_uri_len, "/info.html");
			/* e.g. for large uploads to slow flash over a fast connection, you should
         manually update the rx window. That way, a sender can only send a full
         tcp window at a time. If this is required, set 'post_aut_wnd' to 0.
         We do not need to throttle upload speed here, so: */
			*post_auto_wnd = 1;
			return ERR_OK;
		}
	}
	return ERR_VAL;
}

err_t
httpd_post_receive_data(void *connection, struct pbuf *p)
{
	err_t ret;

	LWIP_ASSERT("NULL pbuf", p != NULL);

	if (current_connection == connection) {


		/* not returning ERR_OK aborts the connection, so return ERR_OK unless the
       connection is unknown */
		ret = ERR_OK;
	} else {
		ret = ERR_VAL;
	}

	/* this function must ALWAYS free the pbuf it is passed or it will leak memory */
	pbuf_free(p);

	return ret;
}

void
httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len)
{
	/* default page is "login failed" */
	snprintf(response_uri, response_uri_len, "/404.html");
	if (current_connection == connection) {
		if (valid_connection == connection) {
			/* login succeeded */
			snprintf(response_uri, response_uri_len, "/info.html");
		}
		current_connection = NULL;
		valid_connection = NULL;
	}
}

