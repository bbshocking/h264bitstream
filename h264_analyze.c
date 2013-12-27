/* 
 * h264bitstream - a library for reading and writing H.264 video
 * Copyright (C) 2005-2007 Auroras Entertainment, LLC
 * 
 * Written by Alex Izvorski <aizvorski@gmail.com>
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "h264_stream.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#define BUFSIZE 32*1024*1024

#if (defined(__GNUC__) || (defined(WIN32)))
#define HAVE_GETOPT_LONG

#include <getopt.h>


static struct option long_options[] =
{
    { "probe",   no_argument, NULL, 'p'},
    { "output",  required_argument, NULL, 'o'},
    { "help",    no_argument,       NULL, 'h'},
    { "verbose", required_argument, NULL, 'v'},
};
#endif

static char options[] =
"\t-o output file, defaults to test.264\n"
"\t-p print information regarding this stream\n"
"\t-v print more info\n"
"\t-h print this message and exit\n";

void h264_usage( )
{

    fprintf( stderr, "h264_analyze, version 0.2.0\n");
    fprintf( stderr, "Analyze H.264 bitstreams in Annex B format\n");
    fprintf( stderr, "h264_usage: \n");

    fprintf( stderr, "h264_analyze [options] <input bitstream>\noptions:\n%s\n", options);
}

int h264_test(char *infile);

int main(int argc, char *argv[])
{
    FILE* infile;

    uint8_t* buf = (uint8_t*)malloc( BUFSIZE );

    h264_stream_t* h = h264_new();

    if (argc < 2) { h264_usage(); return EXIT_FAILURE; }

    int opt_verbose = 1;
    int opt_probe = 0;

#ifdef HAVE_GETOPT_LONG
    int c;
    int long_options_index;
    extern char* optarg;
    extern int   optind;

    while ( ( c = getopt_long( argc, argv, "o:p:hv:t", long_options, &long_options_index) ) != -1 )
    {
        switch ( c )
        {
            case 'o':
                if (h264_dbgfile == NULL) { h264_dbgfile = fopen( optarg, "wt"); }
                break;
            case 'p':
                opt_probe = 1;
                opt_verbose = 0;
                break;
            case 'v':
                opt_verbose = atoi( optarg );
                break;
            case 't':
                h264_test(argv[optind]);
                return 0;
            case 'h':
            default:
                h264_usage( );
                return 1;
        }
    }

    infile = fopen(argv[optind], "rb");

#else

    infile = fopen(argv[1], "rb");

#endif

    if (infile == NULL) { fprintf( stderr, "!! Error: could not open file: %s \n", strerror(errno)); exit(EXIT_FAILURE); }

    if (h264_dbgfile == NULL) { h264_dbgfile = stdout; }
    

    size_t rsz = 0;
    size_t sz = 0;
    int64_t off = 0;
    uint8_t* p = buf;

    int nal_start, nal_end;
	int nal_no = 0, ret = 0;

    while (1)
    {
        rsz = fread(buf + sz, 1, BUFSIZE - sz, infile);
        if (rsz == 0)
        {
            if (ferror(infile)) { fprintf( stderr, "!! Error: read failed: %s \n", strerror(errno)); break; }
            break;  // if (feof(infile)) 
        }

        sz += rsz;

        while (((ret = h264_find_nal_unit(p, sz, &nal_start, &nal_end)) > 0) || (ret == -1 && sz > 0))
        {
            ++nal_no;
            if ( opt_verbose > 0 )
            {
               fprintf( h264_dbgfile, "!! Found NAL %lld at offset %lld (0x%04llX), size %lld (0x%04llX) \n",
                      (long long int)(nal_no),
                      (long long int)(off + (p - buf) + nal_start),
                      (long long int)(off + (p - buf) + nal_start),
                      (long long int)(nal_end - nal_start),
                      (long long int)(nal_end - nal_start) );
            }

            p += nal_start;
            h264_read_nal_unit(h, p, nal_end - nal_start);

            if ( opt_probe && h->nal->nal_unit_type == NAL_UNIT_TYPE_SPS )
            {
                // print codec parameter, per RFC 6381.
                int constraint_byte = h->sps->constraint_set0_flag << 7;
                constraint_byte = h->sps->constraint_set1_flag << 6;
                constraint_byte = h->sps->constraint_set2_flag << 5;
                constraint_byte = h->sps->constraint_set3_flag << 4;
                constraint_byte = h->sps->constraint_set4_flag << 3;
                constraint_byte = h->sps->constraint_set4_flag << 3;

                fprintf( h264_dbgfile, "codec: avc1.%02X%02X%02X\n",h->sps->profile_idc, constraint_byte, h->sps->level_idc );

                // TODO: add more, move to h264_stream (?)
                break; // we've seen enough, bailing out.
            }

            if ( opt_verbose > 0 )
            {
                fprintf( h264_dbgfile, "XX ");
                h264_debug_bytes(p-4, nal_end - nal_start + 4 >= 16 ? 16: nal_end - nal_start + 4);

                h264_debug_nal(h, h->nal);
            }

            p += (nal_end - nal_start);
            sz -= nal_end;
        }

        // if no NALs found in buffer, discard it
        if (p == buf) 
        {
            fprintf( stderr, "!! Did not find any NALs between offset %lld (0x%04llX), size %lld (0x%04llX), discarding \n",
                   (long long int)off, 
                   (long long int)off, 
                   (long long int)off + sz, 
                   (long long int)off + sz);

            p = buf + sz;
            sz = 0;
        }

        memmove(buf, p, sz);
        off += p - buf;
        p = buf;
    }

    h264_free(h);
    free(buf);

    fclose(h264_dbgfile);
    fclose(infile);

    return 0;
}

int h264_test(char *filename)
{
    fprintf(stdout, "test h264bitstream...\n");
    
    FILE* infile;
    FILE* outfile;
    const int nal_prefix_size = 4;
    char nal_prefix[nal_prefix_size]= {0,0,0,1};

    uint8_t* buf = (uint8_t*)malloc( BUFSIZE );
    uint8_t* wbuf = (uint8_t*)malloc( BUFSIZE );

    h264_stream_t* rh = h264_new();
    h264_stream_t* wh = h264_new();

    infile = fopen(filename, "rb");
    outfile = fopen("out.264", "wb");

    if ((infile == NULL) || (outfile == NULL)) { fprintf( stderr, "!! Error: could not open file: %s \n", strerror(errno)); exit(EXIT_FAILURE); }


    size_t rsz = 0, wsz = BUFSIZE;
    size_t sz = 0, sz2 = 0;
    int64_t off = 0;
    uint8_t* p = buf, *wp = wbuf;

    int nal_start, nal_end;
	int nal_no = 0, ret = 0;

    while (1)
    {
        rsz = fread(buf + sz, 1, BUFSIZE - sz, infile);
        if (rsz == 0)
        {
            if (ferror(infile)) { fprintf( stderr, "!! Error: read failed: %s \n", strerror(errno)); break; }
            break;  // if (feof(infile)) 
        }

        sz += rsz;

        while ((ret = h264_find_nal_unit(p, sz, &nal_start, &nal_end)) > 0) 
        {
            ++nal_no;
            
            p += nal_start;
            h264_peek_nal_unit(rh, p, nal_end - nal_start);
            
            memcpy(wp, nal_prefix, nal_prefix_size);
            wp  += nal_prefix_size;
            wsz -= nal_prefix_size;
            
            switch(rh->nal->nal_unit_type)
            {
                 case NAL_UNIT_TYPE_AUD:                
                    h264_read_nal_unit(rh, p, nal_end - nal_start);
                    
                    sz2 = h264_write_nal_unit(rh, wp, wsz);
                    
                    fprintf(stdout, "AUD...\n");
                    break;
                 
                 case NAL_UNIT_TYPE_SPS:                
                    h264_read_nal_unit(rh, p, nal_end - nal_start);
                    
                    sz2 = h264_write_nal_unit(rh, wp, wsz);
                    
                    fprintf(stdout, "SPS...\n");
                    break;
                case NAL_UNIT_TYPE_PPS:
                    h264_read_nal_unit(rh, p, nal_end - nal_start);
                    
                    sz2 = h264_write_nal_unit(rh, wp, wsz);
                    fprintf(stdout, "PPS...\n");
                    break;
                case NAL_UNIT_TYPE_CODED_SLICE_IDR:
                case NAL_UNIT_TYPE_CODED_SLICE_NON_IDR:
                    h264_read_nal_unit(rh, p, nal_end - nal_start);
                    
                    sz2 = h264_write_nal_unit(rh, wp, wsz);
                    memcpy(wp + sz2, p+sz2, nal_end - (nal_start + sz2));
                    sz2 = nal_end - nal_start;
                    fprintf(stdout, "IDR...\n");
                    break;
                default:
                    break;
            }
            
            p += (nal_end - nal_start);
            sz -= nal_end;

            wp += sz2;
            wsz -= sz2;
        }
        
        // if no NALs found in buffer, discard it
        if (p == buf) 
        {
            fprintf( stderr, "!! Did not find any NALs between offset %lld (0x%04llX), size %lld (0x%04llX), discarding \n",
                (long long int)off, 
                (long long int)off, 
                (long long int)off + sz, 
                (long long int)off + sz);
            
            p = buf + sz;
            sz = 0;
        }
        
        memmove(buf, p, sz);
        off += p - buf;
        p = buf;
    }

    fwrite(wbuf, 1, BUFSIZE-wsz, outfile);

    h264_free(rh);
    h264_free(wh);

    free(buf);
    free(wbuf);

    return 0;
}