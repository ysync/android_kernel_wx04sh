/*
 * Copyright (C) 2011 Sharp.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * include
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/binfmts.h>
#include <linux/crypto.h>
#include <linux/scatterlist.h>
#include <linux/delay.h>
#include <sharp/sh_smem.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <linux/fdtable.h>

/*
 * define
 */
// #define USES_UEVENT_NOTIFY
#define DIGEST_SIZE 64
#define DGSTMGRD_SIG "dgstmgrd:"
#define DGSTMGRD_SIG_SIZE (9)
#define DGSTMGRD_KEY_LENGTH (32)
#define DGSTMGRD_DATALIST_LENGTH (8192)
#define DGSTMGRD_PACKAGE_STORE_MAX_LEN (32)
#define sphinx_printk if(0)printk

#define LOCAL_SPHINX_PATH_APPPROCESS	"/system/bin/app_process"
#define LOCAL_SPHINX_SUFFIX_APK		".apk"
#define LOCAL_SPHINX_DIR_SYSTEM		"/system/"
#define LOCAL_SPHINX_DIR_FRAMEWORK		"/system/framework/"
#define LOCAL_SPHINX_DIR_SYSTEMBIN		"/system/bin/"
#define LOCAL_SPHINX_DIR_SYSTEMSHBIN	"/system/shbin/"
#define LOCAL_SPHINX_DIR_VENDORBIN		"/system/vendor/bin/"
#define LOCAL_SPHINX_DIR_SYSTEMAPP		"/system/app/"
#define LOCAL_SPHINX_DIR_VENDORAPP		"/system/vendor/app/"
#define LOCAL_SPHINX_DIR_SYSTEMLIB		"/system/lib/"
#define LOCAL_SPHINX_DIR_VENDORLIB		"/system/vendor/lib/"
#define LOCAL_SPHINX_DIR_DATADATA		"/data/data/"
#define LOCAL_SPHINX_DIR_DATAAPP		"/data/app/"
#define LOCAL_SPHINX_DIR_DATAAPPPRIVATE	"/data/app-private/"

#define CREATE_KOBJECT(dir, file)										\
	static void dir##_release(struct kobject *kobj);					\
	static ssize_t dir##_show(struct kobject *kobj, struct attribute *attr, char *buf); \
	static ssize_t dir##_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len);	\
	static struct kobject dir##_kobj;									\
	static struct kobj_attribute dir##_attribute = __ATTR(file, 0644, NULL, NULL);	\
	static struct attribute *dir##_default_attrs[] = {					\
		&dir##_attribute.attr,											\
		NULL,	/* need to NULL terminate the list of attributes */		\
	};																	\
	static struct sysfs_ops dir##_sysfs_ops = {						\
		.show = dir##_show,											\
		.store = dir##_store,											\
	};																	\
	static struct kobj_type dir##_ktype = {							\
		.release = dir##_release,										\
		.sysfs_ops = &dir##_sysfs_ops,									\
		.default_attrs = dir##_default_attrs,							\
	};																	\
	static kobj_data dir##_data = {										\
		.name = #dir,													\
		.kobj = &dir##_kobj,											\
		.ktype = &dir##_ktype,											\
	};																	\

/*
 * enum
 */
typedef enum
{
	SPHINX_LOAD_PROCESS_ELF,
	SPHINX_LOAD_PROCESS_LINKER,
	SPHINX_LOAD_PROCESS_DEX,
	SPHINX_UNLOAD_PROCESS,
	SPHINX_KILL_PROCESS,
} sphinx_type;

typedef struct
{
	sphinx_type type;
	pid_t pid;
	unsigned char digest[DIGEST_SIZE];
	char name[128];
	char name2[128];
} sphinx_data;

typedef struct _sphinx_node
{
	sphinx_data data;
	struct _sphinx_node *prev;
} sphinx_node;
 
typedef struct
{
	sphinx_node *head;
	sphinx_node *tail;
} sphinx_queue_struct;

typedef struct
{
	const char *name;
	struct kobject *kobj;
	struct kobj_type *ktype;
} kobj_data;

typedef struct
{
	void* next;
	char buffer[256];
} sphinx_proven_struct;

/*
 * extern
 */
extern void presenter_set_readable(void);
int sphinx_elf_unregister_process(pid_t pid);

/*
 * static
 */
static sphinx_proven_struct* sphinx_proven_processes = NULL;
static pid_t digest_manager_pid = -1;
static pid_t package_held_pid = -1;
static struct kset *sphinx_kset;
static sphinx_queue_struct sphinx_queue = {NULL, NULL};

static int sphinx_queue_put(sphinx_data *data);
static int sphinx_queue_get(sphinx_data *data);
static int sphinx_notify_uevent(sphinx_data *data);
static int sphinx_create_kobject(kobj_data *data);

/*
 * object
 */
CREATE_KOBJECT(elfloader, digestsender);
CREATE_KOBJECT(provenprocess, pids);
CREATE_KOBJECT(secure, key);
CREATE_KOBJECT(package, name);

static DEFINE_MUTEX( sphinx_mutex );

/*
 * function
 */
pid_t sphinx_get_digest_manager_pid(void)
{
	return digest_manager_pid;
}

int sphinx_elf_unregister_process(pid_t pid)
{
	int ret;
	sphinx_data data;
	sphinx_proven_struct* curr;

	memset(&data, 0x00, sizeof(sphinx_data));

	data.type = SPHINX_UNLOAD_PROCESS;
	data.pid = pid;

	ret = sphinx_notify_uevent(&data);

	if(pid == digest_manager_pid)
	{
		digest_manager_pid = 0;

		if(sphinx_proven_processes != NULL)
		{
			for(curr = sphinx_proven_processes; curr != NULL; curr = (sphinx_proven_struct*)curr->next)
			{
				if(strstr(curr->buffer, DGSTMGRD_SIG) == curr->buffer)
				{
					memset(curr->buffer, 0, 256);
					sprintf(curr->buffer, "%s0", DGSTMGRD_SIG);
				}
			}
		}
	}

	return ret;
}

static int sphinx_queue_put(sphinx_data *data)
{
	sphinx_node *new_queue = NULL;

	mutex_lock(&sphinx_mutex);

	new_queue = (sphinx_node*)kmalloc(sizeof(sphinx_node), GFP_KERNEL);
	if(new_queue == NULL){
		mutex_unlock(&sphinx_mutex);
		return -ENOMEM;
	}

	memcpy(&new_queue->data, data, sizeof(sphinx_data));
	new_queue->prev = NULL;

	if(sphinx_queue.tail == NULL){
		sphinx_queue.tail = new_queue;
	}
	else{
		sphinx_queue.head->prev = new_queue;
	}
	sphinx_queue.head = new_queue;

	mutex_unlock(&sphinx_mutex);

	return 0;
}
 
static int sphinx_queue_get(sphinx_data *data)
{
	sphinx_node *last_queue = NULL;

	mutex_lock(&sphinx_mutex);

	last_queue = sphinx_queue.tail;
	if(last_queue == NULL){
		mutex_unlock(&sphinx_mutex);
		return -ENOENT;
	}

	sphinx_queue.tail = last_queue->prev;
	memcpy(data, last_queue, sizeof(sphinx_data));

	kfree(last_queue);

	mutex_unlock(&sphinx_mutex);

	return 0;
}

static int sphinx_notify_uevent(sphinx_data *data)
{
	int ret;

	ret = sphinx_queue_put(data);
	if(ret){
		sphinx_printk(KERN_ALERT "sphinx_digest queue_put: %d\n", ret);
		return ret;
	}

#ifdef USES_UEVENT_NOTIFY
	kobject_uevent(&elfloader_kobj, KOBJ_CHANGE);
#else
	presenter_set_readable();
#endif /* USES_UEVENT_NOTIFY */

	return 0;
}

static void elfloader_release(struct kobject *kobj)
{
	kfree(kobj);
}

static ssize_t elfloader_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int ret;

	if(current->pid != digest_manager_pid) return 0;

	ret = sphinx_queue_get((sphinx_data*)buf);
	if(ret) return 0;

	return sizeof(sphinx_data);
}

static ssize_t elfloader_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
{
	int ret;
	sphinx_data *data = (sphinx_data*)buf;

	data->pid = current->pid;
	memset(data->name2, 0, 128);	
	strncpy(data->name2, current->comm, 127);

	ret = sphinx_notify_uevent(data);
	if(ret) return 0;

	return len;
}

static void provenprocess_release(struct kobject *kobj)
{
	kfree(kobj);
}

static ssize_t provenprocess_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sphinx_proven_struct* curr;
	
	for(curr = sphinx_proven_processes; curr != NULL; curr = (sphinx_proven_struct*)curr->next)
	{
		sprintf(buf + ret, "%s\n", curr->buffer);
		ret += (strlen(curr->buffer) + 1);
	}

	return ret;
}

static ssize_t provenprocess_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
{
	sphinx_proven_struct* prev;
	sphinx_proven_struct* curr;
	sphinx_proven_struct* node;
	int n;

	if(current->pid != 1)return 0;

	if(strchr(buf, ':') == NULL)return 0;

	n = (uint32_t)strchr(buf, ':') - (uint32_t)buf;

	if(n < 0)return 0;

	if(sphinx_proven_processes != NULL)
	{
		for(curr = sphinx_proven_processes, prev = sphinx_proven_processes; curr != NULL; curr = (sphinx_proven_struct*)curr->next)
		{
			if(memcmp(buf, curr->buffer, n) == 0)return 0;
			
			prev = curr;
		}

		node = (sphinx_proven_struct*)kmalloc(sizeof(sphinx_proven_struct), GFP_KERNEL);

		if(node == NULL)return 0;

		memset(node, 0, sizeof(sphinx_proven_struct));
		memcpy(node->buffer, buf, len);
		
		prev->next = (void*)node;
	}
	else
	{
		sphinx_proven_processes = (sphinx_proven_struct*)kmalloc(sizeof(sphinx_proven_struct), GFP_KERNEL);

		if(sphinx_proven_processes == NULL)return 0;

		memset(sphinx_proven_processes, 0, sizeof(sphinx_proven_struct));
		memcpy(sphinx_proven_processes->buffer, buf, len);
	}

	if((digest_manager_pid == -1) && (strstr(buf, DGSTMGRD_SIG) == buf))
	{
		const char* ptr = buf + DGSTMGRD_SIG_SIZE;
		char tmp[256];
		char* ptmp = &tmp[0];
		
		memset(tmp, 0, 256);
		memcpy(tmp, ptr, len - DGSTMGRD_SIG_SIZE);
		
		for(digest_manager_pid = 0; *ptmp != '\0' && *ptmp >= '0' && *ptmp <= '9'; ptmp++)
		{
			digest_manager_pid = 10 * digest_manager_pid + (*ptmp - '0');
		}
	}

	return len;
}

static void secure_release(struct kobject *kobj)
{
	kfree(kobj);
}

static ssize_t secure_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	char buff[32];
	sharp_smem_common_type *p_sharp_smem_common_type;

	sphinx_printk("secure_show()\n");

	if(current->pid != digest_manager_pid)
	{
		return 0;
	}

	memset(buff, 0, 32);

	p_sharp_smem_common_type = sh_smem_get_common_address();

	if( p_sharp_smem_common_type != NULL )
	{
		memcpy(buff, p_sharp_smem_common_type->shsecure_PassPhrase, 32);
		memset(p_sharp_smem_common_type->shsecure_PassPhrase, 0, 32);

		memcpy(buf, buff, 32);
	}

	sphinx_printk("secure_show() : 32\n");

	return 32;
}

static ssize_t secure_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
{
	return 0;
}

static void package_release(struct kobject *kobj)
{
	kfree(kobj);
}

static char* sphinx_detect_binary(struct task_struct* t, char* pathbuf)
{
	char* path = NULL;

	sphinx_printk("miyabi_detect_binary()\n");

	if(pathbuf != NULL && t->mm != NULL && t->mm->exe_file != NULL)
	{
		path = d_path(&t->mm->exe_file->f_path, pathbuf, PATH_MAX);

		if(path != NULL && strcmp(path, LOCAL_SPHINX_PATH_APPPROCESS) == 0)
		{
			struct files_struct *files = t->files;
			struct fdtable *fdt;

			if(files != NULL)
			{
				spin_lock(&files->file_lock);

				fdt = files_fdtable(files);

				if(fdt != NULL)
				{
					int j;
					int found = 0;

					for(j = 0; j < fdt->max_fds; j++)
					{
						struct file* f = fdt->fd[j];

						if(f != NULL && f->f_dentry != NULL && f->f_dentry->d_inode != NULL && f->f_dentry->d_inode->i_mode & S_IFREG)
						{
							path = d_path(&f->f_path, pathbuf, PATH_MAX);

							if(path != NULL && strstr(path, LOCAL_SPHINX_DIR_FRAMEWORK) == NULL)
							{
								char* dot = strrchr(path, '.');

								if(dot != NULL && strstr(dot, LOCAL_SPHINX_SUFFIX_APK) != NULL)
								{
									found = 1;

									break;
								}
							}
						}
					}

					if(found == 0)
					{
						path = pathbuf;

						strcpy(path, LOCAL_SPHINX_PATH_APPPROCESS);
					}
				}
				else
				{
					path = NULL;
				}

				spin_unlock(&files->file_lock);
			}
			else
			{
				path = NULL;
			}
		}
	}
	
	sphinx_printk("miyabi_detect_binary() : [%s] %d, %d\n", path, t->pid, t->cred->uid);

	return path;
}

static char* sphinx_detect_package(struct task_struct* t, char* buffer)
{
	char* package = NULL;

	int res = 0;
	unsigned int len;
	struct mm_struct *mm = get_task_mm(t);
	if (!mm)
		goto out;
	if (!mm->arg_end)
		goto out_mm;

 	len = mm->arg_end - mm->arg_start;
 
	if (len > PAGE_SIZE)
		len = PAGE_SIZE;
 
	res = access_process_vm(t, mm->arg_start, buffer, len, 0);

	if (res > 0 && buffer[res-1] != '\0' && len < PAGE_SIZE) {
		len = strnlen(buffer, res);
		if (len < res) {
		    res = len;
		} else {
			len = mm->env_end - mm->env_start;
			if (len > PAGE_SIZE - res)
				len = PAGE_SIZE - res;
			res += access_process_vm(t, mm->env_start, buffer+res, len, 0);
			res = strnlen(buffer, res);
		}
	}
out_mm:
	mmput(mm);
out:
	if(res > 0) package = buffer;

	return package;
}

static ssize_t package_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct task_struct* process;
	struct mm_struct* mm;
	struct vm_area_struct* vm_area;
	struct file* file;
	int i;
	char *packbuf = NULL, *pack = NULL;
	char *binbuf = NULL, *bin = NULL;
	char *pathbuf = NULL, *path = NULL;
	char* p_data = NULL;
	int data_len = 0;
	int failed = 0;

	if(package_held_pid == -1) return 0;
	if(current->tgid != digest_manager_pid) return 0;

	sphinx_printk("held pid 2: %d\n", package_held_pid);

	do
	{
		packbuf = kmalloc(PATH_MAX, GFP_KERNEL);

		if(packbuf == NULL) break;

		binbuf = kmalloc(PATH_MAX, GFP_KERNEL);

		if(binbuf == NULL) break;

		pathbuf = kmalloc(PATH_MAX, GFP_KERNEL);

		if(pathbuf == NULL) break;

		p_data = kmalloc(DGSTMGRD_DATALIST_LENGTH, GFP_KERNEL);

		if(p_data == NULL) break;

		/* --- */

		memset(packbuf, 0, PATH_MAX);
		memset(binbuf, 0, PATH_MAX);
		memset(pathbuf, 0, PATH_MAX);
		memset(p_data, 0, PATH_MAX);

		read_lock(&tasklist_lock);

		do
		{
			process = find_task_by_vpid(package_held_pid);

			if(process == NULL) break;
			if(process->mm == NULL) break;
			if(process->mm->mmap == NULL) break;

			bin = sphinx_detect_binary(process, binbuf);

			if(bin == NULL) break;

			if(!(	strstr(bin, LOCAL_SPHINX_DIR_SYSTEMBIN) == bin ||
				strstr(bin, LOCAL_SPHINX_DIR_SYSTEMSHBIN) == bin ||
				strstr(bin, LOCAL_SPHINX_DIR_VENDORBIN) == bin ||
				strstr(bin, LOCAL_SPHINX_DIR_SYSTEMAPP) == bin ||
				strstr(bin, LOCAL_SPHINX_DIR_VENDORAPP) == bin ||
				(strstr(bin, LOCAL_SPHINX_DIR_DATAAPP) == bin && strstr(bin, LOCAL_SPHINX_SUFFIX_APK) != NULL) ||
				(strstr(bin, LOCAL_SPHINX_DIR_DATAAPPPRIVATE) == bin && strstr(bin, LOCAL_SPHINX_SUFFIX_APK) != NULL )
			))
			{
				break;
			}

			if(strcmp(bin, LOCAL_SPHINX_PATH_APPPROCESS) == 0)
			{
				pack = sphinx_detect_package(process, packbuf);

				if(pack == NULL) break;
			}

			mm = process->mm;
			vm_area = mm->mmap;

			for(i = 0; i < mm->map_count; i++, vm_area = vm_area->vm_next)
			{
				if(vm_area == NULL) break;

				if(vm_area->vm_flags & VM_EXEC)
				{
					file = vm_area->vm_file;
		
					if(file != NULL)
					{
						path = d_path(&file->f_path, pathbuf, PATH_MAX);

						if(path == NULL || (long)path == ENAMETOOLONG)
						{
							failed = 1;

							break;
						}
					
						sphinx_printk("path : %s\n", path);

						if(strcmp(path, "/dev/ashmem/dalvik-jit-code-cache") == 0) continue;
						if(strstr(path, LOCAL_SPHINX_DIR_SYSTEM) == path) continue;
						if(strstr(path, LOCAL_SPHINX_DIR_DATADATA) == path && strstr(path, "/lib/") != NULL && strstr(path, ".so") != NULL)
						{
							int l = strlen(path);

							if(data_len + l + 1 >= DGSTMGRD_DATALIST_LENGTH)
							{
								ret = 0;

								break;
							}

							snprintf(p_data + data_len, DGSTMGRD_DATALIST_LENGTH - 1, "\n%s", path);

							data_len = data_len + l + 1;
						}
					}
					else
					{
						const char *name = arch_vma_name(vm_area);

						if(name == NULL)
						{
							if(vm_area->vm_flags & VM_MAYSHARE)
							{
								failed = 1;

								break;
							}

							continue;
						}

						sphinx_printk("name : %s\n", name);

						if(strcmp(name, "[vectors]") != 0)
						{
							failed = 1;

							break;
						}
					}
				}
			}

			if(failed == 1) break;

			ret = strlen(bin);

			if(ret > 0)
			{
				sprintf(buf, "%s", bin);

				if(data_len > 0)
				{
					sprintf(buf + ret, "%s", p_data);

					ret += data_len;
				}
			}
		}
		while(0);

		read_unlock(&tasklist_lock);
	}
	while(0);


	if(pathbuf != NULL)
	{
		kfree(pathbuf);
		pathbuf = NULL;
	}

	if(binbuf != NULL)
	{
		kfree(binbuf);
		binbuf = NULL;
	}

	if(packbuf != NULL)
	{
		kfree(packbuf);
		packbuf = NULL;
	}

	if(p_data != NULL)
	{
		kfree(p_data);
		p_data = NULL;
	}

	/* --- */

	package_held_pid = -1;

	sphinx_printk("package_show returns : %d\n", ret);

	return ret;
}

static ssize_t package_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
{
	char tmp[DGSTMGRD_PACKAGE_STORE_MAX_LEN + 1];
	char* ptmp = &tmp[0];
	pid_t tmp_pid;

	if(len > DGSTMGRD_PACKAGE_STORE_MAX_LEN) return 0;
	if(package_held_pid != -1) return 0;
	if(current->tgid != digest_manager_pid) return 0;
	
	memset(tmp, 0, DGSTMGRD_PACKAGE_STORE_MAX_LEN + 1);
	memcpy(tmp, buf, len);

	for(tmp_pid = 0; *ptmp != '\0' && *ptmp >= '0' && *ptmp <= '9'; ptmp++)
	{
		tmp_pid = 10 * tmp_pid + (*ptmp - '0');
	}

	package_held_pid = tmp_pid;

	sphinx_printk("held pid : %d\n", package_held_pid);

	return len;
}

static int sphinx_create_kobject(kobj_data *data)
{
	int ret;

	/*
	 * As we have a kset for this kobject, we need to set it before calling
	 * the kobject core.
	 */
	data->kobj->kset = sphinx_kset;

	/*
	 * Initialize and add the kobject to the kernel.  All the default files
	 * will be created here.  As we have already specified a kset for this
	 * kobject, we don't have to set a parent for the kobject, the kobject
	 * will be placed beneath that kset automatically.
	 */
	ret = kobject_init_and_add(data->kobj, data->ktype, NULL, "%s", data->name);
	if(ret) kobject_put(data->kobj);

	/*
	 * We are always responsible for sending the uevent that the kobject
	 * was added to the system.
	 */
	if(!ret) kobject_uevent(data->kobj, KOBJ_ADD);

	return ret;
}

static int __init sphinx_init(void)
{
	int ret;

	/* Create a simple kobject with the name of "sphinx" located under /sys/kernel/ */
	sphinx_kset = kset_create_and_add("digestmanager", NULL, kernel_kobj);
	if(!sphinx_kset) return -ENOMEM;

	ret = sphinx_create_kobject(&elfloader_data);
	ret = sphinx_create_kobject(&provenprocess_data);
	ret = sphinx_create_kobject(&secure_data);
	ret = sphinx_create_kobject(&package_data);

	return ret;
}

static void __exit sphinx_exit(void)
{
	kset_unregister(sphinx_kset);
}

module_init(sphinx_init);
module_exit(sphinx_exit);
MODULE_LICENSE("GPL2");
MODULE_AUTHOR("SHARP");
