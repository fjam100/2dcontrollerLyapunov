/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <string.h>
#include "lcmtypes/bot_param_request_t.h"

static int __bot_param_request_t_hash_computed;
static int64_t __bot_param_request_t_hash;

int64_t __bot_param_request_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __bot_param_request_t_get_hash)
            return 0;

    const __lcm_hash_ptr cp = { p, (void*)__bot_param_request_t_get_hash };
    (void) cp;

    int64_t hash = 0xa686a0e0f882d897LL
         + __int64_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __bot_param_request_t_get_hash(void)
{
    if (!__bot_param_request_t_hash_computed) {
        __bot_param_request_t_hash = __bot_param_request_t_hash_recursive(NULL);
        __bot_param_request_t_hash_computed = 1;
    }

    return __bot_param_request_t_hash;
}

int __bot_param_request_t_encode_array(void *buf, int offset, int maxlen, const bot_param_request_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int bot_param_request_t_encode(void *buf, int offset, int maxlen, const bot_param_request_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __bot_param_request_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __bot_param_request_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __bot_param_request_t_encoded_array_size(const bot_param_request_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __int64_t_encoded_array_size(&(p[element].utime), 1);

    }
    return size;
}

int bot_param_request_t_encoded_size(const bot_param_request_t *p)
{
    return 8 + __bot_param_request_t_encoded_array_size(p, 1);
}

int __bot_param_request_t_decode_array(const void *buf, int offset, int maxlen, bot_param_request_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __bot_param_request_t_decode_array_cleanup(bot_param_request_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_decode_array_cleanup(&(p[element].utime), 1);

    }
    return 0;
}

int bot_param_request_t_decode(const void *buf, int offset, int maxlen, bot_param_request_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __bot_param_request_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __bot_param_request_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int bot_param_request_t_decode_cleanup(bot_param_request_t *p)
{
    return __bot_param_request_t_decode_array_cleanup(p, 1);
}

int __bot_param_request_t_clone_array(const bot_param_request_t *p, bot_param_request_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_clone_array(&(p[element].utime), &(q[element].utime), 1);

    }
    return 0;
}

bot_param_request_t *bot_param_request_t_copy(const bot_param_request_t *p)
{
    bot_param_request_t *q = (bot_param_request_t*) malloc(sizeof(bot_param_request_t));
    __bot_param_request_t_clone_array(p, q, 1);
    return q;
}

void bot_param_request_t_destroy(bot_param_request_t *p)
{
    __bot_param_request_t_decode_array_cleanup(p, 1);
    free(p);
}

int bot_param_request_t_publish(lcm_t *lc, const char *channel, const bot_param_request_t *p)
{
      int max_data_size = bot_param_request_t_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = bot_param_request_t_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _bot_param_request_t_subscription_t {
    bot_param_request_t_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void bot_param_request_t_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    bot_param_request_t p;
    memset(&p, 0, sizeof(bot_param_request_t));
    status = bot_param_request_t_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding bot_param_request_t!!!\n", status);
        return;
    }

    bot_param_request_t_subscription_t *h = (bot_param_request_t_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    bot_param_request_t_decode_cleanup (&p);
}

bot_param_request_t_subscription_t* bot_param_request_t_subscribe (lcm_t *lcm,
                    const char *channel,
                    bot_param_request_t_handler_t f, void *userdata)
{
    bot_param_request_t_subscription_t *n = (bot_param_request_t_subscription_t*)
                       malloc(sizeof(bot_param_request_t_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 bot_param_request_t_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg bot_param_request_t LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int bot_param_request_t_subscription_set_queue_capacity (bot_param_request_t_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int bot_param_request_t_unsubscribe(lcm_t *lcm, bot_param_request_t_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe bot_param_request_t_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

