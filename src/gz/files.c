#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <set/set.h>
#include <vector/vector.h>
#include "files.h"
#include "menu.h"
#include "osk.h"
#include "resource.h"
#include "sys.h"

#define FILE_VIEW_ROWS    14

/* params */
static enum get_file_mode   gf_mode;
static char                *gf_suffix;
static int                  gf_suffix_length;
static get_file_callback_t  gf_callback_proc;
static void                *gf_callback_data;
/* data */
static struct menu          gf_menu;
static struct vector        gf_dir_state;
static struct set           gf_dir_entries;
static _Bool                gf_untitled;
static _Bool                gf_dirty_name;
/* menus */
static struct menu_item    *gf_reset;
static struct menu_item    *gf_location;
static struct menu_item    *gf_name;
static struct menu_item    *gf_accept;
static struct menu_item    *gf_clear;
static struct menu_item    *gf_mkdir;
static struct menu_item    *gf_scroll_up;
static struct menu_item    *gf_scroll_down;
static struct menu_item    *gf_files[FILE_VIEW_ROWS];

struct dir_state
{
  int     scroll;
  int     index;
};

struct dir_entry
{
  char    name[256];
  _Bool   dir;
  int     tile;
  char    text[32];
  int     anim_state;
};

static _Bool stricmp(const char *a, const char *b)
{
  while (*a && *b) {
    char ca = *a++;
    char cb = *b++;
    if (ca >= 'a' && ca <= 'z')
      ca += 'A' - 'a';
    if (cb >= 'a' && cb <= 'z')
      cb += 'A' - 'a';
    if (ca != cb)
      return 0;
  }
  return *a == *b;
}

static _Bool update_list(void)
{
  /* clear entries */
  vector_clear(&gf_dir_entries.container);
  /* update location */
  DIR *dir = opendir(".");
  if (!dir) {
    if (errno == ENODEV)
      strcpy(gf_location->text, "no disk");
    else if (errno == ENOENT)
      strcpy(gf_location->text, "no file system");
    else {
      strncpy(gf_location->text, strerror(errno), 31);
      gf_location->text[31] = 0;
    }
    return 0;
  }
  getcwd(gf_location->text, 32);
  gf_location->text[31] = 0;
  /* enumerate and sort entries */
  struct dirent *dirent;
  while ((dirent = readdir(dir))) {
    if ((dirent->d_name[0] == '.' && strcmp(dirent->d_name, "..") != 0) ||
        !(dirent->mode & S_IRUSR))
    {
      continue;
    }
    _Bool dir = ((dirent->mode & S_IFMT) == S_IFDIR);
    int nl = strlen(dirent->d_name);
    int sl = gf_suffix_length;
    if (!dir && (nl < sl || !stricmp(&dirent->d_name[nl - sl], gf_suffix)))
      continue;
    struct dir_entry entry;
    strcpy(entry.name, dirent->d_name);
    entry.dir = dir;
    if (strcmp(dirent->d_name, "..") == 0) {
      entry.tile = 0;
      strcpy(entry.text, "back");
    }
    else {
      if (entry.dir)
        entry.tile = 1;
      else
        entry.tile = 2;
      memcpy(entry.text, dirent->d_name, 32);
      if (nl > 31)
        strcpy(&entry.text[28], "...");
    }
    entry.anim_state = 0;
    set_insert(&gf_dir_entries, &entry);
  }
  closedir(dir);
  return 1;
}

static void update_view(_Bool enable, _Bool select)
{
  if (enable) {
    int y = 3;
    if (gf_mode == GETFILE_SAVE || gf_mode == GETFILE_SAVE_PREFIX_INC) {
      menu_item_enable(gf_name);
      menu_item_enable(gf_mkdir);
      menu_item_enable(gf_accept);
      menu_item_enable(gf_clear);
      gf_name->y = y++;
      gf_accept->y = gf_clear->y = y++;
    }
    else if (gf_mode == GETFILE_LOAD) {
      menu_item_disable(gf_name);
      menu_item_disable(gf_mkdir);
      menu_item_disable(gf_accept);
      menu_item_disable(gf_clear);
    }
    menu_item_enable(gf_scroll_up);
    menu_item_enable(gf_scroll_down);
    gf_scroll_up->y = y;
    gf_scroll_down->y = y + FILE_VIEW_ROWS - 1;
    struct dir_state *ds = vector_at(&gf_dir_state, 0);
    _Bool selected = 0;
    for (int i = 0; i < FILE_VIEW_ROWS; ++i) {
      int index = ds->scroll + i;
      struct menu_item *item = gf_files[i];
      if (index < gf_dir_entries.container.size) {
        item->y = y++;
        menu_item_enable(item);
        if (select && index == ds->index) {
          menu_select(&gf_menu, item);
          selected = 1;
        }
      }
      else
        menu_item_disable(item);
    }
    if (select && !selected) {
      if (gf_mode == GETFILE_LOAD)
        menu_select(&gf_menu, gf_reset);
      else
        menu_select(&gf_menu, gf_name);
    }
  }
  else {
    menu_item_disable(gf_name);
    menu_item_disable(gf_mkdir);
    menu_item_disable(gf_accept);
    menu_item_disable(gf_clear);
    menu_item_disable(gf_scroll_up);
    menu_item_disable(gf_scroll_down);
    for (int i = 0; i < FILE_VIEW_ROWS; ++i)
      menu_item_disable(gf_files[i]);
    if (select)
      menu_select(&gf_menu, gf_reset);
  }
}

static int get_next_prefix_number()
{
  DIR *dir = opendir(".");
  if (!dir)
    return 0;

  int max_num_found = -1;
  int sl = strlen(gf_suffix);

  /* enumerate entries */
  struct dirent *dirent;
  while ((dirent = readdir(dir))) {
    if (!(dirent->mode & S_IRUSR))
      continue;
    int nl = strlen(dirent->d_name);
    if (nl < sl || !stricmp(&dirent->d_name[nl - sl], gf_suffix))
      continue;

    int cur_num;
    int ret = sscanf(dirent->d_name, "%d", &cur_num);
    if (ret == EOF || ret < 1)
      continue;
    if (cur_num > max_num_found)
      max_num_found = cur_num;
  }

  closedir(dir);
  return max_num_found + 1;
}

static void set_name(const char *name, _Bool dirty)
{
  gf_dirty_name |= dirty;
  if (!name || strlen(name) == 0) {
    strcpy(gf_name->text, "untitled");
    gf_untitled = 1;
  }
  else {
    if (gf_dirty_name || gf_mode != GETFILE_SAVE_PREFIX_INC)
      strncpy(gf_name->text, name, 31);
    else {
      int ignore, prefix_length;
      sscanf(name, "%d%n", &ignore, &prefix_length);
      int prefix = get_next_prefix_number();
      snprintf(gf_name->text, 32, "%03d%s", prefix, name + prefix_length);
    }
    gf_name->text[31] = 0;
    gf_untitled = 0;
  }
}

static int overwrite_prompt_proc(int option_index, void *data)
{
  char *path = data;
  if (option_index != -1) {
    menu_return(&gf_menu);
    if (option_index == 0 && !gf_callback_proc(path, gf_callback_data))
      menu_return(&gf_menu);
  }
  free(path);
  return 1;
}

static void return_path(const char *name)
{
  char *path = malloc(PATH_MAX);
  if (path && getcwd(path, PATH_MAX)) {
    int dl = strlen(path);
    int nl = strlen(name);
    if (dl + 1 + nl + gf_suffix_length < PATH_MAX) {
      path[dl] = '/';
      strcpy(&path[dl + 1], name);
      strcpy(&path[dl + 1 + nl], gf_suffix);
      if ((gf_mode == GETFILE_SAVE || gf_mode == GETFILE_SAVE_PREFIX_INC) &&
          stat(path, NULL) == 0)
      {
        char prompt[48];
        sprintf(prompt, "'%.31s' exists", name);
        menu_prompt(&gf_menu, prompt, "overwrite\0""cancel\0", 1,
                    overwrite_prompt_proc, path);
        return;
      }
      if (!gf_callback_proc(path, gf_callback_data))
        menu_return(&gf_menu);
    }
  }
  if (path)
    free(path);
}

static int file_enter_proc(struct menu_item *item,
                           enum menu_switch_reason reason)
{
  int row = (int)item->data;
  struct dir_state *ds = vector_at(&gf_dir_state, 0);
  int index = ds->scroll + row;
  if (index < gf_dir_entries.container.size) {
    struct dir_entry *entry = set_at(&gf_dir_entries, index);
    entry->anim_state = 0;
  }
  return 0;
}

static int file_draw_proc(struct menu_item *item,
                          struct menu_draw_params *draw_params)
{
  int row = (int)item->data;
  struct dir_state *ds = vector_at(&gf_dir_state, 0);
  struct dir_entry *entry = set_at(&gf_dir_entries, ds->scroll + row);
  if (entry->anim_state > 0) {
    ++draw_params->x;
    ++draw_params->y;
    entry->anim_state = (entry->anim_state + 1) % 3;
  }
  int cw = menu_get_cell_width(item->owner, 1);
  struct gfx_texture *texture = resource_get(RES_ICON_FILE);
  struct gfx_sprite sprite =
  {
    texture,
    entry->tile,
    draw_params->x + (cw - texture->tile_width) / 2,
    draw_params->y - (gfx_font_xheight(draw_params->font) +
                      texture->tile_height + 1) / 2,
    1.f, 1.f,
  };
  gfx_mode_set(GFX_MODE_COLOR, GPACK_RGBA8888(0xFF, 0xFF, 0xFF,
                                              draw_params->alpha));
  gfx_sprite_draw(&sprite);
  gfx_mode_set(GFX_MODE_COLOR, GPACK_RGB24A8(draw_params->color,
                                             draw_params->alpha));
  gfx_printf(draw_params->font, draw_params->x + cw * 2, draw_params->y,
             "%s", entry->text);
  return 1;
}

static int file_activate_proc(struct menu_item *item)
{
  int row = (int)item->data;
  struct dir_state *ds = vector_at(&gf_dir_state, 0);
  int index = ds->scroll + row;
  struct dir_entry *entry = set_at(&gf_dir_entries, index);
  entry->anim_state = 1;
  if (entry->dir) {
    if (chdir(entry->name))
      return 0;
    if (strcmp(entry->name, "..") == 0)
      vector_erase(&gf_dir_state, 0, 1);
    else {
      struct dir_state *ds = vector_at(&gf_dir_state, 0);
      ds->index = index;
      ds = vector_insert(&gf_dir_state, 0, 1, NULL);
      ds->scroll = 0;
      ds->index = 0;
    }

    /* Update the prefix */
    if (gf_mode == GETFILE_SAVE_PREFIX_INC)
      set_name(gf_name->text, 0);

    update_view(update_list(), 1);
  }
  else {
    struct dir_state *ds = vector_at(&gf_dir_state, 0);
    ds->index = index;
    int l = strlen(entry->name) - gf_suffix_length;
    char *name = malloc(l + 1);
    if (name) {
      memcpy(name, entry->name, l);
      name[l] = 0;
      if (gf_mode == GETFILE_SAVE || gf_mode == GETFILE_SAVE_PREFIX_INC) {
        set_name(name, 1);
        menu_select(&gf_menu, gf_accept);
      }
      else
        return_path(name);
      free(name);
    }
  }
  return 1;
}

static int file_nav_proc(struct menu_item *item, enum menu_navigation nav) {
  int row = (int)item->data;
  int n_entries = gf_dir_entries.container.size;
  if (row == 0 && nav == MENU_NAVIGATE_UP) {
    struct dir_state *ds = vector_at(&gf_dir_state, 0);
    --ds->scroll;
    if (ds->scroll + FILE_VIEW_ROWS > n_entries)
      ds->scroll = n_entries - FILE_VIEW_ROWS;
    if (ds->scroll < 0) {
      if (n_entries < FILE_VIEW_ROWS) {
        ds->scroll = 0;
        menu_select(item->owner, gf_files[n_entries - 1]);
      }
      else {
        ds->scroll = n_entries - FILE_VIEW_ROWS;
        menu_select(item->owner, gf_files[FILE_VIEW_ROWS - 1]);
      }
    }
    return 1;
  }
  else if ((row == FILE_VIEW_ROWS - 1 ||
            row == gf_dir_entries.container.size - 1) &&
           nav == MENU_NAVIGATE_DOWN)
  {
    struct dir_state *ds = vector_at(&gf_dir_state, 0);
    ++ds->scroll;
    int index = ds->scroll + row;
    if (index == gf_dir_entries.container.size){
      ds->scroll = 0;
      menu_select(item->owner, gf_files[0]);
    }
    return 1;
  }
  return 0;
}

static _Bool dir_entry_comp(void *a, void *b)
{
  struct dir_entry *da = a;
  struct dir_entry *db = b;
  if (strcmp(da->name, "..") == 0)
    return 1;
  if (da->dir && !db->dir)
    return 1;
  if (!da->dir && db->dir)
    return 0;
  char *sa = da->name;
  char *sb = db->name;
  int d;
  while (*sa && *sb) {
    char ca = *sa++;
    char cb = *sb++;
    /* number comparison */
    if (ca >= '0' && ca <= '9' && cb >= '0' && cb <= '9') {
      char *na = sa - 1;
      char *nb = sb - 1;
      while (*na == '0')
        ++na;
      while (*nb == '0')
        ++nb;
      while (*sa >= '0' && *sa <= '9')
        ++sa;
      while (*sb >= '0' && *sb <= '9')
        ++sb;
      d = (sa - na) - (sb - nb);
      if (d < 0)
        return 1;
      if (d > 0)
        return 0;
      while (na != sa && nb != sb) {
        d = *na++ - *nb++;
        if (d < 0)
          return 1;
        if (d > 0)
          return 0;
      }
      continue;
    }
    /* case-insensitive character comparison */
    if (ca >= 'a' && ca <= 'z')
      ca += 'A' - 'a';
    if (cb >= 'a' && cb <= 'z')
      cb += 'A' - 'a';
    d = ca - cb;
    if (d < 0)
      return 1;
    if (d > 0)
      return 0;
  }
  /* length comparison */
  while (*sa)
    ++sa;
  while (*sb)
    ++sb;
  d = (sa - da->name) - (sb - db->name);
  if (d < 0)
    return 1;
  else
    return 0;
}

static int osk_callback_proc(const char *str, void *data)
{
  set_name(str, strcmp(str, gf_name->text) != 0);
  gf_menu.selector = gf_accept;
  return 0;
}

static int name_activate_proc(struct menu_item *item)
{
  menu_get_osk_string(item->owner, gf_untitled ? NULL : item->text,
                      osk_callback_proc, NULL);
  return 1;
}

static void accept_proc(struct menu_item *item, void *data)
{
  return_path(gf_name->text);
}

static void clear_proc(struct menu_item *item, void *data)
{
  set_name(NULL, 1);
}

static void reset_proc(struct menu_item *item, void *data)
{
  sys_reset();
  vector_clear(&gf_dir_state);
  struct dir_state *ds = vector_insert(&gf_dir_state, 0, 1, NULL);
  ds->scroll = 0;
  ds->index = 0;
  update_view(update_list(), 1);
}

static void scroll_up_proc(struct menu_item *item, void *data)
{
  struct dir_state *ds = vector_at(&gf_dir_state, 0);
  --ds->scroll;
  if (ds->scroll < 0)
    ds->scroll = 0;
}

static void scroll_down_proc(struct menu_item *item, void *data)
{
  struct dir_state *ds = vector_at(&gf_dir_state, 0);
  ++ds->scroll;
  int n_entries = gf_dir_entries.container.size;
  if (ds->scroll + FILE_VIEW_ROWS > n_entries)
    ds->scroll = n_entries - FILE_VIEW_ROWS;
  if (ds->scroll < 0)
    ds->scroll = 0;
}

static int mkdir_osk_callback_proc(const char *str, void *data)
{
  if (*str == '\0')
    return 0;
  else if (mkdir(str, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH)) {
    menu_prompt(&gf_menu, strerror(errno), "return\0", 0, NULL, NULL);
    return 1;
  }
  else {
    menu_return(&gf_menu);
    update_view(update_list(), 1);
    return 1;
  }
}

static void mkdir_proc(struct menu_item *item, void *data)
{
  menu_get_osk_string(item->owner, NULL, mkdir_osk_callback_proc, NULL);
}

static void gf_menu_init(void)
{
  static _Bool ready = 0;
  if (!ready) {
    ready = 1;
    /* initialize data */
    vector_init(&gf_dir_state, sizeof(struct dir_state));
    set_init(&gf_dir_entries, sizeof(struct dir_entry), dir_entry_comp);
    struct dir_state *ds = vector_insert(&gf_dir_state, 0, 1, NULL);
    struct gfx_texture *file_icons = resource_get(RES_ICON_FILE);
    ds->scroll = 0;
    ds->index = 0;
    /* initialize menus */
    struct menu *menu = &gf_menu;
    menu_init(menu, MENU_NOVALUE, MENU_NOVALUE, MENU_NOVALUE);
    menu->selector = menu_add_submenu(menu, 0, 0, NULL, "return");
    gf_reset = menu_add_button(menu, 0, 1, "reset disk", reset_proc, NULL);
    gf_location = menu_add_static(menu, 0, 2, NULL, 0xC0C0C0);
    gf_location->text = malloc(32);
    gf_mkdir = menu_add_button_icon(menu, 0, 3, file_icons, 3, 0xFFFFFF,
                                    mkdir_proc, NULL);
    gf_name = menu_item_add(menu, 2, 2, NULL, 0xFFFFFF);
    gf_name->text = malloc(32);
    gf_name->text[0] = 0;
    gf_name->activate_proc = name_activate_proc;
    gf_accept = menu_add_button(menu, 0, 4, "accept", accept_proc, NULL);
    gf_clear = menu_add_button(menu, 7, 4, "clear", clear_proc, NULL);
    for (int i = 0; i < FILE_VIEW_ROWS; ++i) {
      struct menu_item *item = menu_item_add(menu, 2, 5 + i, NULL, 0xFFFFFF);
      item->data = (void *)i;
      item->enter_proc = file_enter_proc;
      item->draw_proc = file_draw_proc;
      item->activate_proc = file_activate_proc;
      item->navigate_proc = file_nav_proc;
      gf_files[i] = item;
    }
    struct gfx_texture *t_arrow = resource_get(RES_ICON_ARROW);
    gf_scroll_up = menu_add_button_icon(menu, 0, 5,
                                        t_arrow, 0, 0xFFFFFF,
                                        scroll_up_proc, NULL);
    gf_scroll_down = menu_add_button_icon(menu, 0, 5 + FILE_VIEW_ROWS - 1,
                                          t_arrow, 1, 0xFFFFFF,
                                          scroll_down_proc, NULL);
  }
  update_view(update_list(), 1);
}

void menu_get_file(struct menu *menu, enum get_file_mode mode,
                   const char *defname, const char *suffix,
                   get_file_callback_t callback_proc, void *callback_data)
{
  gf_mode = mode;
  gf_dirty_name = 0;
  if (gf_suffix)
    free(gf_suffix);
  gf_suffix = malloc(strlen(suffix) + 1);
  if (!gf_suffix)
    return;
  strcpy(gf_suffix, suffix);
  gf_suffix_length = strlen(gf_suffix);
  gf_callback_proc = callback_proc;
  gf_callback_data = callback_data;
  gf_menu_init();
  set_name(defname, 0);
  menu_enter(menu, &gf_menu);
}
