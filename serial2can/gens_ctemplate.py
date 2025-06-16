import sys
import datetime


class CTemplate:

    file_name = ' '
    source_name = ' '
    header_name = ' '
    _time = datetime.datetime.now().strftime('%Y.%m.%d')
    file_comments = {'@copyright':'Copyright (c) 2022 DAVID HU All rights reserved. Licensed under the MIT License (the "License");\r\n\
* you may not use this file except in compliance with the License.\r\n\
* You may obtain a copy of the License in the file LICENSE\r\n\
* Unless required by applicable law or agreed to in writing, software\r\n\
* distributed under the License is distributed on an "AS IS" BASIS,\r\n\
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\r\n\
* See the License for the specific language governing permissions and\r\n\
* limitations under the License.',
                      '@note':' ',
                      '@date':_time,
                      '@version':'0.1',
                      '@brief':' ',
                      '@author':'David Hu (hmd_hubei_cn@163.com)',
                      '@file':" ",
    }

    comments_order = ('@file','@author','@brief','@version','@date','@note','@copyright')

    def __init__(self,s):
        self.file_name = s
        self.source_name = s + ".c"
        self.header_name = s + '.h'
    
    def generate_comments_at_top(self,name):
        self.file_comments['@file'] = name

        comments = ('/**\n')

        #find max length of string
        max_len = 0
        for s in self.comments_order:
            if(len(s) > max_len):
                max_len = len(s)
         
        for k in self.comments_order:
            align_space_amount = max_len - len(k) + 4
            align_space = align_space_amount * ' '
            comments += ('* '+ k + align_space + self.file_comments[k] + '\n')

        comments += ('**/\n\n')

        return comments

    def generate_comments_in_the_end(self):
        return ("/********************* (C) COPYRIGHT DAVID HU *******END OF FILE ********/\n")

    def create_source(self):
        fh = open(self.source_name,mode = 'w',encoding='utf-8')
        cm = self.generate_comments_at_top(self.source_name)
        cm += ("#include \"%s\"\n" %self.header_name) 
        cm += ("\n"*5)
        cm += self.generate_comments_in_the_end()
        fh.write(cm)
        fh.close()

    def create_header(self):
        fh = open(self.header_name,mode = 'w',encoding='utf-8')
        cm = self.generate_comments_at_top(self.header_name)
        cm += "#ifndef __%s_H__\n" %self.file_name.upper()
        cm += "#define __%s_H__\n" %self.file_name.upper()
        cm += ("\n"*2)
        cm += "#ifdef __cplusplus\n"
        cm += "extern \"C\"{\n"
        cm += "#endif\n"
        cm += ("\n"*5)
        cm += "#ifdef __cplusplus\n"
        cm += "}\n"
        cm += "#endif\n"
        cm += ("\n"*2)
        cm += "#endif //__%s_H__\n"%self.file_name.upper() 
        cm += self.generate_comments_in_the_end()
        fh.write(cm)
        fh.close()

    def create_template_pairs(self):
        self.create_source()
        self.create_header()


if __name__ == '__main__':
        if len(sys.argv) != 2:
            sys.stderr.write("please input corret file name")
        else:
            s = sys.argv[1]
            ct = CTemplate(s)
            ct.create_template_pairs()
