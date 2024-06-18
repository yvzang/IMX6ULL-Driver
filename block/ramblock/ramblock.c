#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <uapi/asm-generic/errno-base.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <uapi/linux/hdreg.h>
#include <linux/spinlock.h>


#define RAMBLOCK_SIZE					(2*1024*1024)
#define RAMBLOCK_NAME					"ramblock"
#define RAMBLOCK_MINOR					3

struct ramblock_dev{
	int major;
	uint8_t *ram_mem;
	struct gendisk *gendisk;
	struct request_queue *queue;
	spinlock_t spinlock;
};

static struct ramblock_dev ramblock;

static int ramblk_open(struct block_device *dev, fmode_t mod){
	printk("open ram block device.\r\n");
	return 0;
}

static void ramblk_release(struct gendisk *gendisk, fmode_t mod){
	printk("release ram block device.\r\n");
}

int ramblk_getgeo(struct block_device *dev, struct hd_geometry *geometry){
	geometry->heads = 2;
	geometry->cylinders = 32;
	geometry->sectors = RAMBLOCK_SIZE / (2 * 32 * 512);
	return 0;
}

static struct block_device_operations ramblk_ops = {
	.open = ramblk_open,
	.release = ramblk_release,
	.getgeo = ramblk_getgeo
};

#if 0
void request_transfer(struct request *req){
	uint32_t start = blk_rq_pos(req)<<9;
	uint32_t length = blk_rq_cur_bytes(req);

	void *buffer = bio_data(req->bio);
	if(rq_data_dir(req) == READ){
		memcpy(buffer, ramblock.ram_mem+start, length);
	}
	else{
		memcpy(ramblock.ram_mem+start, buffer, length);
	}
}

static void request_handler(struct request_queue *q){
	struct request *rq;
	int err=0;
	rq = blk_fetch_request(q);
	while(rq !=NULL){
		/* 处理请求 */
		request_transfer(rq);
		if(!__blk_end_request_cur(rq, err))
			rq = blk_fetch_request(q);
	}
}
#endif

void ramblock_make_request_fn(struct request_queue *rq, struct bio *bio){
	struct bvec_iter iter;
	struct bio_vec bvec;
	uint32_t offset, len;
	uint8_t *buffer;

	offset = bio->bi_iter.bi_sector << 9;
	bio_for_each_segment(bvec, bio, iter){
		buffer = page_address(bvec.bv_page) + bvec.bv_offset;
		len = bvec.bv_len;

		if(bio_data_dir(bio)==READ){
			memcpy(buffer, ramblock.ram_mem+offset, len);
		}
		else if(bio_data_dir(bio) == WRITE){
			memcpy(ramblock.ram_mem+offset, buffer, len);
		}
		offset += len;
	}
	set_bit(BIO_UPTODATE, &bio->bi_flags);
	bio_endio(bio, 0);
}

static int __init ramblock_init(void){
	int ret = 0;
	/* 分配虚拟内存 */
	ramblock.ram_mem = kzalloc(RAMBLOCK_SIZE, GFP_KERNEL);
	if(!ramblock.ram_mem){
		printk("fails to allocate virtual memory.\r\n");
		ret = -ENOSPC;
		goto fail_alloc;
	}

	/* 注册block_dev */
	ramblock.major = register_blkdev(0, RAMBLOCK_NAME);
	if(!ramblock.major){
		printk("failst to register block device.\r\n");
		goto fail_register_blkdev;
	}
	printk("ram block device major: %d\r\n", ramblock.major);

	/* 分配gendisk */
	ramblock.gendisk = alloc_disk(RAMBLOCK_MINOR);
	if(!ramblock.gendisk){
		printk("fails to allocate gendisk.\r\n");
		ret = -ENOSPC;
		goto fail_alloc_gendisk;
	}
	/* 初始化自旋锁 */
	spin_lock_init(&ramblock.spinlock);

#if 0
	/* 申请并初始化request_queue */
	ramblock.queue = blk_init_queue(request_handler, &ramblock.spinlock);
	if(!ramblock.queue){
		printk("fails to init queue.\r\n");
		ret = -EINTR;
		goto fail_alloc_gendisk;
	}
#endif
	/* 申请request_queue */
	ramblock.queue = blk_alloc_queue(GFP_KERNEL);
	if(!ramblock.queue){
		printk("fails to allocate request queue.\r\n");
		ret = -ENOSPC;
		goto fail_alloc_gendisk;
	}
	blk_queue_make_request(ramblock.queue, ramblock_make_request_fn);

	/* 添加gendisk */
	ramblock.gendisk->major = ramblock.major;
	ramblock.gendisk->first_minor = 0;
	sprintf(ramblock.gendisk->disk_name, RAMBLOCK_NAME);
	ramblock.gendisk->queue = ramblock.queue;
	ramblock.gendisk->private_data = (void *)&ramblock;
	ramblock.gendisk->fops = &ramblk_ops;
	set_capacity(ramblock.gendisk, RAMBLOCK_SIZE/512);
	/* gendisk加入内核 */
	add_disk(ramblock.gendisk);
	return 0;

fail_alloc_gendisk:
	del_gendisk(ramblock.gendisk);

fail_register_blkdev:
	unregister_blkdev(ramblock.major, RAMBLOCK_NAME);

fail_alloc:
	kfree(ramblock.ram_mem);
	return ret;
}

static void __exit ramblock_exit(void){
	/* 删除gendisk */
	del_gendisk(ramblock.gendisk);
	put_disk(ramblock.gendisk);
	/* 清除队列 */
	blk_cleanup_queue(ramblock.queue);
	/* 注销ramblock_dev*/
	unregister_blkdev(ramblock.major, RAMBLOCK_NAME);
	/* 释放内存 */
	kfree(ramblock.ram_mem);
}

module_init(ramblock_init);
module_exit(ramblock_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("yuzang");