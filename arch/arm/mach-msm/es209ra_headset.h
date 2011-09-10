/* SEMC:modified */
/* 
   Header file of Audio Jack Driver

   Copyright (C) 2009 Sony Ericsson Mobile Communications Japan, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License, version 2, as
   published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#ifndef LINUX_ES209RA_AUDIO_JACK
#define LINUX_ES209RA_AUDIO_JACK

struct es209ra_headset_platform_data {
	const char * keypad_name;
	unsigned int gpio_detout;
	unsigned int gpio_detin;
	unsigned int wait_time;
};

void es209ra_audio_jack_button_handler(int key);

#endif
