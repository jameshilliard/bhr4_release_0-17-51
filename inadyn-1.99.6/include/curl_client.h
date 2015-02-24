/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/
#include <stdlib.h>
#include <string.h>
#include <curl/curl.h>

#define HTTP_DEFAULT_PORT 80

struct MemoryStruct {
	char *memory;
	size_t size;
};

void *myrealloc(void *ptr, size_t size);
size_t WriteMemoryCallback(void *ptr, size_t size, size_t nmemb, void *data);
int curl_client(char *url, char *userpwd, char *ua, char* buffer, size_t buffersize, char *proxy_host, int proxy_port, char* hostname);
