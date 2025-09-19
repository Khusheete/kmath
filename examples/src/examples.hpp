// Copyright © 2025 Souchet Ferdinand (aka. Khusheete)
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the “Software”), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#pragma once

void *rotor_motor_init();
void rotor_motor_run(void *p_data);
void rotor_motor_cleanup(void *p_data);


void *camera_init();
void camera_run(void *p_data);
void camera_cleanup(void *p_data);


void *motor_transforms_init();
void motor_transforms_run(void *p_data);
void motor_transforms_cleanup(void *p_data);


void *pga_visualization_init();
void pga_visualization_run(void *p_data);
void pga_visualization_cleanup(void *p_data);


void *oklab_interpolation_init();
void oklab_interpolation_run(void *p_data);
void oklab_interpolation_cleanup(void *p_data);


void *ease_function_init();
void ease_function_run(void *p_data);
void ease_function_cleanup(void *p_data);
