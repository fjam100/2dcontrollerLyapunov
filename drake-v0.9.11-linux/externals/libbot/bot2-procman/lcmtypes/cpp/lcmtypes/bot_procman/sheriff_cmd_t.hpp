/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __bot_procman_sheriff_cmd_t_hpp__
#define __bot_procman_sheriff_cmd_t_hpp__

#include <string>

namespace bot_procman
{

class sheriff_cmd_t
{
    public:
        std::string name;
        std::string nickname;
        std::string group;
        int32_t    desired_runid;
        int8_t     force_quit;
        int32_t    sheriff_id;
        int8_t     auto_respawn;

    public:
        inline int encode(void *buf, int offset, int maxlen) const;
        inline int getEncodedSize() const;
        inline int decode(const void *buf, int offset, int maxlen);
        inline static int64_t getHash();
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static int64_t _computeHash(const __lcm_hash_ptr *p);
};

int sheriff_cmd_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int sheriff_cmd_t::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int sheriff_cmd_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t sheriff_cmd_t::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* sheriff_cmd_t::getTypeName()
{
    return "sheriff_cmd_t";
}

int sheriff_cmd_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    char* name_cstr = (char*) this->name.c_str();
    tlen = __string_encode_array(buf, offset + pos, maxlen - pos, &name_cstr, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    char* nickname_cstr = (char*) this->nickname.c_str();
    tlen = __string_encode_array(buf, offset + pos, maxlen - pos, &nickname_cstr, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    char* group_cstr = (char*) this->group.c_str();
    tlen = __string_encode_array(buf, offset + pos, maxlen - pos, &group_cstr, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->desired_runid, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->force_quit, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->sheriff_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_encode_array(buf, offset + pos, maxlen - pos, &this->auto_respawn, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int sheriff_cmd_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    int32_t __name_len__;
    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &__name_len__, 1);
    if(tlen < 0) return tlen; else pos += tlen;
    if(__name_len__ > maxlen - pos) return -1;
    this->name.assign(((const char*)buf) + offset + pos, __name_len__ - 1);
    pos += __name_len__;

    int32_t __nickname_len__;
    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &__nickname_len__, 1);
    if(tlen < 0) return tlen; else pos += tlen;
    if(__nickname_len__ > maxlen - pos) return -1;
    this->nickname.assign(((const char*)buf) + offset + pos, __nickname_len__ - 1);
    pos += __nickname_len__;

    int32_t __group_len__;
    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &__group_len__, 1);
    if(tlen < 0) return tlen; else pos += tlen;
    if(__group_len__ > maxlen - pos) return -1;
    this->group.assign(((const char*)buf) + offset + pos, __group_len__ - 1);
    pos += __group_len__;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->desired_runid, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->force_quit, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->sheriff_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_decode_array(buf, offset + pos, maxlen - pos, &this->auto_respawn, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int sheriff_cmd_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += this->name.size() + 4 + 1;
    enc_size += this->nickname.size() + 4 + 1;
    enc_size += this->group.size() + 4 + 1;
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int8_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __boolean_encoded_array_size(NULL, 1);
    return enc_size;
}

int64_t sheriff_cmd_t::_computeHash(const __lcm_hash_ptr *)
{
    int64_t hash = 0x3fbd1729a0aee378LL;
    return (hash<<1) + ((hash>>63)&1);
}

}

#endif
