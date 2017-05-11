/*
 * at_parser.c
 *
 * Created: 09/05/2017 21:44:55
 *  Author: jan.rune.herheim
 */ 

#include <stdint.h>
#include <string.h>

#define AT_HEAD                                         "AT"
#define AT_HEAD_SIZE                                    2
#define AT_CMS_ERROR                                    "+CMS ERROR:"
#define AT_CMS_ERROR_SIZE                               11
#define AT_HEADER_SIZE                                  15
#define AT_STORAGE_SIZE                                 50
#define AT_TRANSFER_SIZE                                1024
#define DEFAULT_TIMEOUT                                 500

uint8_t at_cmd_storage_used = 10;

typedef void ( *at_cmd_cb )( char *response );

static uint32_t _at_hash( char *cmd )
{
	uint16_t ch     = 0;
	uint32_t hash   = 4321;

	while( ( ch = *( cmd++ ) ) )
	hash = ( ( hash << 2 ) + hash ) + ch;

	return hash;
}

typedef struct
{
	uint8_t                hash;
	uint8_t                timeout;
	at_cmd_cb               getter;
	at_cmd_cb               setter;
	at_cmd_cb               tester;
	at_cmd_cb               executer;

} at_cmd_t;

static at_cmd_t at_cmd_storage [ AT_STORAGE_SIZE ];


typedef enum
{
	AT_CMD_UNKNOWN                              = 0,
	AT_CMD_GET                                  = 1,
	AT_CMD_SET                                  = 2,
	AT_CMD_TEST                                 = 3,
	AT_CMD_EXEC                                 = 4,

}at_type_t;

static at_type_t _at_sub_parse( char *raw_in,
char *clean_out )
{
	uint8_t     c                         = 0;
	uint8_t     end_pos                   = 0;
	uint8_t     set_pos                   = 0;
	uint8_t     get_pos                   = 0;
	uint8_t     start_pos                 = 0;
	char*       tmp_ptr                   = raw_in;
	char        tmp_cmd[ AT_HEADER_SIZE ] = { 0 };

	if( strlen( tmp_ptr ) <= AT_HEAD_SIZE )
	return AT_CMD_UNKNOWN;

	strncpy( tmp_cmd, tmp_ptr, AT_HEADER_SIZE );

	for( c = 0; c < AT_HEADER_SIZE; c++ )
	{
		if( tmp_cmd[ c ] == '\0' )
		{
			if( !end_pos )
			end_pos = c;
			break;
		}

		if( ( tmp_cmd[ c ] == '+') && !start_pos )
		start_pos = c;

		if( ( tmp_cmd[ c ] == '=' ) && !set_pos )
		set_pos = c;

		if( ( tmp_cmd[ c ] == '?' ) && !get_pos )
		get_pos = c;

		if( ( ( tmp_cmd[ c ] == '\r' )  ||
		( tmp_cmd[ c ] == '\n' )  ||
		( tmp_cmd[ c ] == ':' ) ) && !end_pos )
		end_pos = c;
	}

	if( !set_pos && !get_pos )
	{
		strncpy( clean_out, &tmp_cmd[ start_pos ], end_pos - start_pos );
		return AT_CMD_EXEC;

		} else if( !set_pos && get_pos ) {

		strncpy( clean_out, &tmp_cmd[ start_pos ], get_pos - start_pos );
		return AT_CMD_TEST;

		} else if( set_pos && !get_pos ) {

		strncpy( clean_out, &tmp_cmd[ start_pos ], set_pos - start_pos );
		return AT_CMD_SET;

		} else if( set_pos == get_pos - 1 ) {

		strncpy( clean_out, &tmp_cmd[ start_pos ], set_pos - start_pos );
		return AT_CMD_GET;
	}
	return AT_CMD_UNKNOWN;
}

static uint16_t _at_search( char* cmd )
{
	uint16_t i;
	uint8_t tmp_hash = _at_hash( cmd );

	for( i = 0; i < at_cmd_storage_used; i++ )
	if( at_cmd_storage[ i ].hash == tmp_hash )
	return i;

	return 0;
}

void at_parse( char *input, at_cmd_cb *cb, uint8_t *timeout )
{
	at_type_t   cmd_type                    = 0;
	uint16_t    cmd_idx                     = 0;
	char        cmd_temp[ AT_HEADER_SIZE ]  = { 0 };

	if( !( cmd_type = _at_sub_parse( input, cmd_temp ) ) )
	{
		*cb = at_cmd_storage[ 0 ].executer;
		*timeout = at_cmd_storage[ 0 ].timeout;
		return;
	}

	if( !( cmd_idx = _at_search( cmd_temp ) ) )
	{
		*cb = at_cmd_storage[ 0 ].executer;
		*timeout = at_cmd_storage[ 0 ].timeout;
		return;
	}

	switch ( cmd_type )
	{
		case AT_CMD_SET :
		*cb = at_cmd_storage[ cmd_idx ].setter;
		*timeout = at_cmd_storage[ cmd_idx ].timeout;
		break;
		case AT_CMD_GET :
		*cb = at_cmd_storage[ cmd_idx ].getter;
		*timeout = at_cmd_storage[ cmd_idx ].timeout;
		break;
		case AT_CMD_TEST :
		*cb = at_cmd_storage[ cmd_idx ].tester;
		*timeout = at_cmd_storage[ cmd_idx ].timeout;
		break;
		case AT_CMD_EXEC :
		*cb = at_cmd_storage[ cmd_idx ].executer;
		*timeout = at_cmd_storage[ cmd_idx ].timeout;
		break;
		case AT_CMD_UNKNOWN :
		*cb = at_cmd_storage[ 0 ].executer;
		*timeout = at_cmd_storage[ 0 ].timeout;
		break;
	}
	return;
}
