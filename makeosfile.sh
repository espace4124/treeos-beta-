#!/bin/bash
mkdir -p MyOS_Project
cd MyOS_Project

echo "Generation MyOS v2.2 avec curseur 2x2 minimaliste et clic pixel gris..."

# =========================
# 1. BOOTLOADER
# =========================
cat > boot.asm << 'EOF'
[BITS 16]
[ORG 0x7C00]

start:
    xor ax, ax
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, 0x7C00

    mov si, msg_boot
    call print_string

    mov ah, 0x02
    mov al, 20
    mov ch, 0
    mov cl, 2
    mov dh, 0
    mov bx, 0x1000
    int 0x13
    jc disk_error

    jmp 0x1000

disk_error:
    mov si, msg_err
    call print_string
    jmp $

print_string:
    lodsb
    or al, al
    jz .done
    mov ah, 0x0E
    int 0x10
    jmp print_string
.done:
    ret

msg_boot db 'Booting MyOS v2.2...',13,10,0
msg_err  db 'Disk Error',0

times 510-($-$$) db 0
dw 0xAA55
EOF

# =========================
# 2. STAGE2
# =========================
cat > stage2.asm << 'EOF'
[BITS 16]
stage2_entry:
    cli
    lgdt [gdt_descriptor]
    mov eax, cr0
    or eax, 1
    mov cr0, eax
    jmp 0x08:init_pm

[BITS 32]
init_pm:
    mov ax, 0x10
    mov ds, ax
    mov ss, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ebp, 0x90000
    mov esp, ebp
    extern kernel_main
    call kernel_main
    jmp $

gdt_start: dq 0x0
gdt_code: dw 0xFFFF,0x0000,0x9A00,0x00CF
gdt_data: dw 0xFFFF,0x0000,0x9200,0x00CF
gdt_end:

gdt_descriptor: dw gdt_end - gdt_start - 1
dd gdt_start
EOF

# =========================
# 3. KERNEL avec curseur 2x2
# =========================
cat > kernel.c << 'EOF'
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;

#define VGA 0xB8000
#define W 80
#define H 25
#define CURSOR_CHAR 219  // bloc plein ASCII VGA

uint16_t* vga = (uint16_t*)VGA;
int row=0, col=0;
uint8_t color=0x0F;
int ctrl_pressed=0;

// --- Mouse system ---
typedef struct { int x, y; } Mouse;
Mouse mouse = {40, 12};

#define CURSOR_SIZE 2
uint16_t mouse_backup[CURSOR_SIZE][CURSOR_SIZE];

// Curseur 2x2 minimaliste
const char mouse_design[2][3] = {
    "11",  // ligne 0 : deux pixels blancs
    "10"   // ligne 1 : premier pixel blanc, second transparent
};

// Sauvegarde le fond sous le curseur
void save_mouse_background() {
    for(int dy=0; dy<CURSOR_SIZE; dy++){
        for(int dx=0; dx<CURSOR_SIZE; dx++){
            int x = mouse.x + dx;
            int y = mouse.y + dy;
            if(x<0 || x>=W || y<0 || y>=H) continue;
            mouse_backup[dy][dx] = vga[y*W + x];
        }
    }
}

// Restaure le fond
void restore_mouse_background() {
    for(int dy=0; dy<CURSOR_SIZE; dy++){
        for(int dx=0; dx<CURSOR_SIZE; dx++){
            int x = mouse.x + dx;
            int y = mouse.y + dy;
            if(x<0 || x>=W || y<0 || y>=H) continue;
            vga[y*W + x] = mouse_backup[dy][dx];
        }
    }
}

// Dessine le curseur
void draw_mouse_color(uint8_t color){
    for(int dy=0; dy<CURSOR_SIZE; dy++){
        for(int dx=0; dx<CURSOR_SIZE; dx++){
            int x = mouse.x + dx;
            int y = mouse.y + dy;
            if(x<0 || x>=W || y<0 || y>=H) continue;
            if(mouse_design[dy][dx]=='1'){
                vga[y*W + x] = CURSOR_CHAR | (color<<8);
            }
        }
    }
}

// Déplacement du curseur
void mouse_move(int dx,int dy){
    restore_mouse_background();
    mouse.x += dx;
    mouse.y -= dy;
    if(mouse.x<0) mouse.x=0;
    if(mouse.x>=(W-CURSOR_SIZE)) mouse.x=W-CURSOR_SIZE;
    if(mouse.y<0) mouse.y=0;
    if(mouse.y>=(H-CURSOR_SIZE)) mouse.y=H-CURSOR_SIZE;
    save_mouse_background();
    draw_mouse_color(0x0F); // blanc
}

// Flash au clic gauche
void mouse_flash_click(){
    draw_mouse_color(0x07); // gris
    for(int i=0;i<500000;i++) __asm__("nop"); // courte pause
    draw_mouse_color(0x0F); // blanc
}

// --- I/O ---
void outb(uint16_t p,uint8_t d){__asm__("outb %0,%1"::"a"(d),"Nd"(p));}
uint8_t inb(uint16_t p){uint8_t r;__asm__("inb %1,%0":"=a"(r):"Nd"(p));return r;}

// --- Screen ---
void putc(char c){
    if(c=='\b'){ if(col>0) col--; vga[row*W+col]=' '|color<<8; return; }
    vga[row*W+col]=(uint16_t)c|(uint16_t)color<<8;
    col++;
    if(col>=W){col=0; row++;}
    if(row>=H) row=0;
}
void print(const char* s){for(int i=0;s[i];i++){if(s[i]=='\n'){col=0;row++;}else putc(s[i]);}}
void clear(){for(int i=0;i<W*H;i++) vga[i]=' '|color<<8; row=col=0;}

// --- Keyboard ---
char get_char(){static int shift=0;uint8_t sc;const char keymap[] = "\0\0""1234567890-=\0" "\0qwertyuiop[]\n\0" "asdfghjkl;'\0\\" "zxcvbnm,./ ";while(1){while(!(inb(0x64)&1)); sc=inb(0x60); if(sc==0x2A||sc==0x36){shift=1;continue;} if(sc==0xAA||sc==0xB6){shift=0;continue;} if(sc==0x1D){ctrl_pressed=1;continue;} if(sc==0x9D){ctrl_pressed=0;continue;} if(sc&0x80)continue; if(ctrl_pressed && sc==0x10) return 0x11; if(sc==0x1C) return '\n'; if(sc==0x0E) return '\b'; if(sc==0x39) return ' '; if(sc<sizeof(keymap)){char c=keymap[sc]; if(shift && c>='a'&&c<='z') c-=32; return c; } }}

// --- Mouse I/O ---
void mouse_wait(){while(inb(0x64)&2);}
void mouse_write(uint8_t data){mouse_wait(); outb(0x64,0xD4); mouse_wait(); outb(0x60,data);}
uint8_t mouse_read(){while(!(inb(0x64)&1)); return inb(0x60);}
void mouse_init(){mouse_wait(); outb(0x64,0xA8); mouse_wait(); outb(0x64,0x20); mouse_wait(); uint8_t status=inb(0x60)|2; mouse_wait(); outb(0x64,0x60); mouse_wait(); outb(0x60,status); mouse_write(0xF6); mouse_read(); mouse_write(0xF4); mouse_read();}

// --- Mouse test ---
void mousetest_loop(){
    mouse_init();
    print("Souris activee (CTRL+Q pour quitter)\n");
    save_mouse_background();
    draw_mouse_color(0x0F);
    while(1){
        if(inb(0x64)&1){
            uint8_t b1 = mouse_read();
            if(!(b1 & 0x08)) continue;
            uint8_t b2 = mouse_read();
            uint8_t b3 = mouse_read();
            int dx = b2 - (((b1>>4)&1)<<8);
            int dy = b3 - (((b1>>5)&1)<<8);
            if(b1 & 0x40 || b1 & 0x80){ dx = 0; dy = 0; }
            mouse_move(dx, dy);
            if(b1 & 0x01) mouse_flash_click();
        }
        if(inb(0x64)&1){
            uint8_t sc = inb(0x60);
            if(sc == 0x10 && ctrl_pressed) break;
        }
    }
    clear();
}

// --- Sound ---
void play(int f){uint32_t div=1193180/f; outb(0x43,0xB6); outb(0x42,div); outb(0x42,div>>8); outb(0x61,inb(0x61)|3);}
void stop(){outb(0x61,inb(0x61)&0xFC);}
void sleep_ms(int ms){for(int i=0;i<ms*1000;i++) __asm__("nop");}
void startup_sound(){int notes[]={440,660,880}; for(int i=0;i<3;i++){play(notes[i]); sleep_ms(200); stop(); sleep_ms(50);}}

extern void shell_main();
void kernel_main(){ clear(); startup_sound(); print("Systeme initialise.\n"); shell_main(); }
EOF

# =========================
# 4. SHELL
# =========================
cat > shell.c << 'EOF'
extern void print(const char*);
extern void clear();
extern char get_char();
extern void mousetest_loop();

int strcmp(const char*a,const char*b){ while(*a&&(*a==*b)){a++;b++;} return *a-*b; }

char file_content[1024];
char filename[32];
int saved=0;

void shell_main(){
    char buf[64];
    print("Shell v2.2\nTape help\n");
    while(1){
        print("\n> "); int i=0;
        while(1){
            char c=get_char();
            if(c=='\n'){ buf[i]=0; print("\n"); break; }
            if(c=='\b'){ if(i>0){i--; print("\b");} continue; }
            buf[i++]=c;
            char s[2]={c,0}; print(s);
        }
        if(strcmp(buf,"help")==0){ print("edit nom.txt cat clear mousetest shutdown\n"); }
        else if(buf[0]=='e'&&buf[1]=='d'&&buf[2]=='i'&&buf[3]=='t'&&buf[4]==' '){
            int j=5,k=0; while(buf[j]) filename[k++]=buf[j++]; filename[k]=0;
            print("Edition de "); print(filename); print("\nCTRL+Q quitter\n");
            int ptr=0;
            while(1){
                char c=get_char();
                if(c==0x11){ file_content[ptr]=0; saved=1; break; }
                file_content[ptr++]=c;
                char s[2]={c,0}; print(s);
            }
        }
        else if(strcmp(buf,"cat")==0){ if(saved) print(file_content); else print("Aucun fichier\n"); }
        else if(strcmp(buf,"clear")==0){ clear(); }
        else if(strcmp(buf,"mousetest")==0){ clear(); mousetest_loop(); }
        else if(strcmp(buf,"shutdown")==0){ print("Arret...\n"); __asm__("hlt"); }
        else{ print("Commande inconnue\n"); }
    }
}
EOF

# =========================
# LINKER
# =========================
cat > link.ld << 'EOF'
ENTRY(stage2_entry)
SECTIONS{
    . = 0x1000;
    .text : { stage2.o(.text) *(.text) }
    .data : { *(.data) }
    .bss : { *(.bss) }
}
EOF

# =========================
# BUILD SCRIPT
# =========================
cat > build.sh << 'EOF'
#!/bin/bash
nasm -f bin boot.asm -o boot.bin
nasm -f elf32 stage2.asm -o stage2.o
gcc -m32 -ffreestanding -fno-pie -c kernel.c -o kernel.o
gcc -m32 -ffreestanding -fno-pie -c shell.c -o shell.o
ld -m elf_i386 -T link.ld -o kernel.bin stage2.o kernel.o shell.o --oformat binary
cat boot.bin kernel.bin > os-image.bin
truncate -s 1M os-image.bin
qemu-system-i386 -hda os-image.bin
EOF
chmod +x build.sh

echo "================================="
echo "MyOS v2.2 généré avec curseur 2x2 minimaliste (11 / 10) et clic PS/2 fonctionnel."
echo "cd MyOS_Project"
echo "./build.sh"
echo "================================="
