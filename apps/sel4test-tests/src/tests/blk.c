#include <string.h>
#include <stdio.h>

#include <sel4/sel4.h>
#include <vka/capops.h>
#include <allocman/allocman.h>
#include <allocman/vka.h>
#include <allocman/bootstrap.h>
#include <sel4utils/thread.h>
#include <serial_server/parent.h>
#include <serial_server/client.h>
// #include <simple/simple.h>
#include <virtio.h>
#include <virtio_queue.h>
#include <queue.h>
#include <ialloc.h>
#include <storage_info.h>
#include <simple-default/simple-default.h>
#include "../test.h"
#include "../helpers.h"

#define QUEUE_SIZE 1024
#define VIRTQ_NUM_REQUESTS QUEUE_SIZE
#define SERSERV_TEST_ALLOCMAN_PREALLOCATED_MEMSIZE (64 * 1024)
#define SHMEM_DRIVER_KB 4096

// blk driver process
static helper_thread_t blk_driver_thread;

typedef struct blk_driver_boot_info {
    uintptr_t regs_vaddr;
    uintptr_t headers_vaddr;
    uintptr_t headers_paddr;

    uintptr_t meta_vaddr;
    uintptr_t meta_paddr;

    uintptr_t storage_info_vaddr;
    uintptr_t request_shmem_vaddr;
    uintptr_t request_paddr;
    uintptr_t response_shmem_vaddr;
    uintptr_t response_paddr;

} blk_driver_boot_info_t;


// void notified(microkit_channel ch)
// {
//     if (ch == device_resources.irqs[0].id) {
//         handle_irq();
//         microkit_deferred_irq_ack(ch);
//         /*
//          * It is possible that we could not enqueue all requests when being notified
//          * by the virtualiser because we ran out of space, so we try again now that
//          * we have received a response and have resources freed.
//          */
//         handle_request();
//     } else if (ch == config.virt.id) {
//         handle_request();
//     } else {
//         LOG_DRIVER_ERR("received notification from unknown channel: 0x%x\n", ch);
//     }
// }

static size_t bytes_to_size_bits(size_t size_bytes) {
    assert(size_bytes > 0);
    ZF_LOGI("In bytes_to_size_bits, bytes: %d\n", size_bytes);
    int bits = 0;
    size_t tmp = size_bytes;

    if ((tmp & (tmp - 1)) != 0) {
        tmp--;
        while (tmp > 0) {
            tmp >>= 1;
            bits++;
        }
        bits++; 
    } else {
        while (tmp > 1) {
            tmp >>= 1;
            bits++;
        }
    }

    return bits;
}

static void map_and_share_frame(struct env *env, sel4utils_process_t target_process,  uintptr_t vaddr, void **paddr_ptr, size_t size_bytes)
{
    seL4_Error err;
    ZF_LOGI("In map_and_share_frame, bytes: %d\n", size_bytes);
    size_t size_bits = bytes_to_size_bits(size_bytes);
    uint32_t page_bits = size_bits > 12 ? seL4_PageBits : seL4_LargePageBits;
    size_t page_count = size_bytes / BIT(page_bits);
    assert((1UL << size_bits) == size_bytes);
    ZF_LOGI("In map_and_share_frame after reservation\n");
    uintptr_t current_vaddr  = vaddr;
    uintptr_t current_paddr = (uintptr_t)(*paddr_ptr);

    // get frame cap
    for (int i = 0; i < page_count; i++) {
        current_paddr = current_paddr ? (current_paddr + i * BIT(page_bits)) : 0;
        current_vaddr = current_vaddr + i * BIT(page_bits);
        reservation_t reservation = vspace_reserve_range(&env->vspace, BIT(page_bits),
            seL4_ReadWrite, 0, (void **)&current_vaddr);
        assert(reservation.res != NULL);
        vka_object_t frame;
        int err;

        if (current_paddr) {
            err = vka_alloc_frame_at(&env->vka, page_bits, current_paddr, &frame);
        } else {
            err = vka_alloc_frame(&env->vka, page_bits, &frame);
            *paddr_ptr = vka_object_paddr(&env->vka, &frame);
        }
        
        // ZF_LOGI("In for loop alloc frame return error: %d\n",err);
        assert(err == seL4_NoError);
        err = vspace_map_pages_at_vaddr(&env->vspace, &frame.cptr, NULL, (void *)current_vaddr, 1, page_bits, reservation);
        // ZF_LOGI("After vspace_map_pages_at_vaddr return error: %d\n",err);
        assert(err == seL4_NoError);
        err = sel4utils_share_mem_at_vaddr(&env->vspace, &target_process.vspace, (void *)current_vaddr, 1, page_bits, (void *)current_vaddr, reservation);
        // ZF_LOGI("After sel4utils_share_mem_at_vaddr return error: %d\n",err);
        assert(err == seL4_NoError);
    }
}

static void setupMMIO_regs_shmem(struct env *env, sel4utils_process_t target_process, blk_driver_boot_info_t* bootinfo) {
    uintptr_t mmio_regs_phys_addr = 0xa003000;
    size_t size_bytes= 0x1000;
    ZF_LOGI("In setupMMIO_regs_shmem: %d\n", size_bytes);
    map_and_share_frame(env, target_process, bootinfo->regs_vaddr, &mmio_regs_phys_addr, size_bytes);
}


static void setupMMIO_virtio_headers_shmem(struct env *env, sel4utils_process_t target_process, blk_driver_boot_info_t* bootinfo) {
    // uintptr_t mmio_virtio_phys_addr = 0x5fff0000;
    uintptr_t mmio_virtio_phys_addr = 0;
    size_t size_bytes = 0x10000;
    ZF_LOGI("In setupMMIO_virtio_headers_shmem: %d\n", size_bytes);
    map_and_share_frame(env, target_process, bootinfo->headers_vaddr, &mmio_virtio_phys_addr, size_bytes);
    bootinfo->headers_paddr = mmio_virtio_phys_addr;
}

static void setupMMIO_virtio_metadata_shmem(struct env *env, sel4utils_process_t target_process, blk_driver_boot_info_t* bootinfo) {
    // uintptr_t mmio_meta_phys_addr = 0x5fdf0000;
    uintptr_t mmio_meta_phys_addr = 0;
    size_t size_bytes = 0x200000;
    ZF_LOGI("In setupMMIO_virtio_metadata_shmem: %d\n", size_bytes);
    map_and_share_frame(env, target_process, bootinfo->meta_vaddr, &mmio_meta_phys_addr, size_bytes);
    bootinfo->meta_paddr = mmio_meta_phys_addr;
}

static void setup_driver_storage_info_shmem(struct env *env, sel4utils_process_t target_process, blk_driver_boot_info_t* bootinfo) {
    size_t size_bytes = 0x1000;
    uintptr_t mmio_driver_storage_info_phys_addr = 0;
    ZF_LOGI("In setup_driver_storage_info_shmem: %d\n", size_bytes);
    map_and_share_frame(env, target_process, bootinfo->storage_info_vaddr, &mmio_driver_storage_info_phys_addr, size_bytes);
}

static void setup_driver_queue_shmem(struct env *env, sel4utils_process_t target_process, blk_driver_boot_info_t* bootinfo) {
    size_t size_bytes = 0x200000;
    uintptr_t mmio_driver_request_phys_addr = 0;
    uintptr_t mmio_driver_response_phys_addr = 0;
    ZF_LOGI("In setup_driver_queue_shmem: %d\n", size_bytes);
    map_and_share_frame(env, target_process, bootinfo->request_shmem_vaddr, &mmio_driver_request_phys_addr, size_bytes);
    map_and_share_frame(env, target_process, bootinfo->response_shmem_vaddr, &mmio_driver_response_phys_addr, size_bytes);
    bootinfo-> request_paddr = mmio_driver_request_phys_addr;
    bootinfo-> response_paddr = mmio_driver_response_phys_addr;

}

static int blk_driver_entry_point(seL4_Word _bootinfo, seL4_Word a1, seL4_Word a2, seL4_Word a3)
{
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");


    blk_driver_boot_info_t *boot_info = (blk_driver_boot_info_t *)_bootinfo;
    uintptr_t requests_paddr = boot_info->request_paddr;
    uintptr_t requests_vaddr = boot_info->request_shmem_vaddr;

    volatile virtio_mmio_regs_t *regs = (volatile virtio_mmio_regs_t *) boot_info->regs_vaddr;

    volatile struct virtq virtq;
    
    blk_queue_handle_t blk_queue;

    uintptr_t virtio_headers_paddr = boot_info->headers_paddr;
    struct virtio_blk_req *virtio_headers = (struct virtio_blk_req *) boot_info->headers_vaddr;

    /*
    * A mapping from virtIO header index in the descriptor virtq ring, to the sDDF ID given
    * in the request. We need this mapping due to out of order operations.
    */
    uint32_t virtio_header_to_id[QUEUE_SIZE];

    /*
    * Due to the out-of-order nature of virtIO, we need a way of allocating indexes in a
    * non-linear way.
    */
    ialloc_t ialloc_desc;
    uint32_t descriptors[QUEUE_SIZE];

    uint16_t last_seen_used = 0;

    /* Block device configuration, populated during initiliastion. */
    volatile struct virtio_blk_config *virtio_config;

    void testfun(void);

    testfun();

    void virtio_blk_init(void)
    {
        // Do MMIO device init (section 4.2.3.1)
        if (!virtio_mmio_check_magic(regs)) {
            LOG_DRIVER_ERR("invalid virtIO magic value!\n");
            assert(false);
        }
    
        if (virtio_mmio_version(regs) != VIRTIO_VERSION) {
            LOG_DRIVER_ERR("not correct virtIO version!\n");
            assert(false);
        }
    
        if (!virtio_mmio_check_device_id(regs, VIRTIO_DEVICE_ID_BLK)) {
            LOG_DRIVER_ERR("not a virtIO block device!\n");
            assert(false);
        }
    
        if (virtio_mmio_version(regs) != VIRTIO_BLK_DRIVER_VERSION) {
            LOG_DRIVER_ERR("driver does not support given virtIO version: 0x%x\n", virtio_mmio_version(regs));
            assert(false);
        }
    
        ialloc_init(&ialloc_desc, descriptors, QUEUE_SIZE);
    
        /* First reset the device */
        regs->Status = 0;
        /* Set the ACKNOWLEDGE bit to say we have noticed the device */
        regs->Status = VIRTIO_DEVICE_STATUS_ACKNOWLEDGE;
        /* Set the DRIVER bit to say we know how to drive the device */
        regs->Status = VIRTIO_DEVICE_STATUS_DRIVER;
    
        virtio_config = (volatile struct virtio_blk_config *)regs->Config;
    #ifdef DEBUG_DRIVER
        virtio_blk_print_config(virtio_config);
    #endif
    
        if (virtio_config->capacity < BLK_TRANSFER_SIZE / VIRTIO_BLK_SECTOR_SIZE) {
            LOG_DRIVER_ERR("driver does not support device capacity smaller than 0x%x bytes"
                           " (device has capacity of 0x%lx bytes)\n",
                           BLK_TRANSFER_SIZE, virtio_config->capacity * VIRTIO_BLK_SECTOR_SIZE);
            assert(false);
        }
    
        /* This driver does not support Read-Only devices, so we always leave this as false */
        blk_storage_info_t *storage_info = config.virt.storage_info.vaddr;
        storage_info->read_only = false;
        storage_info->capacity = (virtio_config->capacity * VIRTIO_BLK_SECTOR_SIZE) / BLK_TRANSFER_SIZE;
        storage_info->cylinders = virtio_config->geometry.cylinders;
        storage_info->heads = virtio_config->geometry.heads;
        storage_info->blocks = virtio_config->geometry.sectors;
        storage_info->block_size = 1;
        storage_info->sector_size = VIRTIO_BLK_SECTOR_SIZE;
    
        /* Finished populating configuration */
        __atomic_store_n(&storage_info->ready, true, __ATOMIC_RELEASE);
    
    #ifdef DEBUG_DRIVER
        uint32_t features_low = regs->DeviceFeatures;
        regs->DeviceFeaturesSel = 1;
        uint32_t features_high = regs->DeviceFeatures;
        uint64_t features = features_low | ((uint64_t)features_high << 32);
        virtio_blk_print_features(features);
    #endif
        /* Select features we want from the device */
        regs->DriverFeatures = 0;
        regs->DriverFeaturesSel = 1;
        regs->DriverFeatures = 0;
    
        regs->Status |= VIRTIO_DEVICE_STATUS_FEATURES_OK;
        if (!(regs->Status & VIRTIO_DEVICE_STATUS_FEATURES_OK)) {
            LOG_DRIVER_ERR("device status features is not OK!\n");
            return;
        }
    
        /* Add virtqueues */
        size_t desc_off = 0;
        size_t avail_off = ALIGN(desc_off + (16 * VIRTQ_NUM_REQUESTS), 2);
        size_t used_off = ALIGN(avail_off + (6 + 2 * VIRTQ_NUM_REQUESTS), 4);
        size_t size = used_off + (6 + 8 * VIRTQ_NUM_REQUESTS);
    
        // Make sure that the metadata region is able to fit all the virtIO specific
        // extra data.
        assert(size <= device_resources.regions[2].region.size);
    
        virtq.num = VIRTQ_NUM_REQUESTS;
        virtq.desc = (struct virtq_desc *)(requests_vaddr + desc_off);
        virtq.avail = (struct virtq_avail *)(requests_vaddr + avail_off);
        virtq.used = (struct virtq_used *)(requests_vaddr + used_off);
    
        assert(regs->QueueNumMax >= VIRTQ_NUM_REQUESTS);
        regs->QueueSel = 0;
        regs->QueueNum = VIRTQ_NUM_REQUESTS;
        regs->QueueDescLow = (requests_paddr + desc_off) & 0xFFFFFFFF;
        regs->QueueDescHigh = (requests_paddr + desc_off) >> 32;
        regs->QueueDriverLow = (requests_paddr + avail_off) & 0xFFFFFFFF;
        regs->QueueDriverHigh = (requests_paddr + avail_off) >> 32;
        regs->QueueDeviceLow = (requests_paddr + used_off) & 0xFFFFFFFF;
        regs->QueueDeviceHigh = (requests_paddr + used_off) >> 32;
        regs->QueueReady = 1;
    
        /* Finish initialisation */
        regs->Status |= VIRTIO_DEVICE_STATUS_DRIVER_OK;
        regs->InterruptACK = VIRTIO_MMIO_IRQ_VQUEUE;
    }


    virtio_blk_init();

    blk_queue_init(&blk_queue, config.virt.req_queue.vaddr, config.virt.resp_queue.vaddr, config.virt.num_buffers);

}



void create_and_share_boot_info(struct env *env, sel4utils_process_t target_process, void** vaddr_to_map)
{
    seL4_Error err;
    ZF_LOGI("in create_and_share_boot_info\n");
    vka_object_t frame_obj;
    err = vka_alloc_frame(&env->vka, seL4_PageBits, &frame_obj);
    assert(err == seL4_NoError);
    ZF_LOGI("After alloc frame\n");
    cspacepath_t frame_path;
    vka_cspace_make_path(&env->vka, frame_obj.cptr, &frame_path);

    reservation_t reservation = vspace_reserve_range(&env->vspace, PAGE_SIZE_4K, seL4_ReadWrite, 0, vaddr_to_map);
    assert(reservation.res != NULL);

    // mapping frame to current process
    vspace_map_pages_at_vaddr(&env->vspace, &frame_path.capPtr, NULL, *vaddr_to_map, 1, seL4_PageBits,  reservation);
    ZF_LOGI("After vspace_map_pages_at_vaddr\n");
    // mapping current vaddr to target vaddr
    err = sel4utils_share_mem_at_vaddr(&env->vspace, &target_process.vspace, *vaddr_to_map, 1, seL4_PageBits, *vaddr_to_map, reservation);
    assert(err == seL4_NoError);
    ZF_LOGI("After sel4utils_share_mem_at_vaddr\n");
}


static void init_blk_driver_server(struct env *env) {
    int error;
    seL4_Word prio = 200;
    ZF_LOGI("in init_blk_driver_server\n");
    // init process
    create_helper_process(env, &blk_driver_thread);
    set_helper_priority(env, &blk_driver_thread, prio);

    // init shared memory
    ZF_LOGI("After create process\n");
    void *vaddr_to_map;
    create_and_share_boot_info(env, blk_driver_thread.process, &vaddr_to_map);
    blk_driver_boot_info_t *driver_bootinfo = (blk_driver_boot_info_t *) vaddr_to_map;
    ZF_LOGI("After create bootinfo, %x\n", vaddr_to_map);
    driver_bootinfo->regs_vaddr = 0x20000000;
    ZF_LOGI("After init regs_vaddr\n");
    driver_bootinfo->headers_vaddr = 0x20001000;
    driver_bootinfo->meta_vaddr = 0x20200000;
    driver_bootinfo->storage_info_vaddr = 0x20400000;
    driver_bootinfo->request_shmem_vaddr = 0x20600000;
    driver_bootinfo->response_shmem_vaddr = 0x20800000;
    driver_bootinfo->request_paddr = 0xa003000;
    ZF_LOGI("After init bootinfo\n");
    setupMMIO_regs_shmem(env, blk_driver_thread.process, driver_bootinfo);
    setupMMIO_virtio_headers_shmem(env, blk_driver_thread.process, driver_bootinfo);
    setupMMIO_virtio_metadata_shmem(env, blk_driver_thread.process, driver_bootinfo);
    setup_driver_storage_info_shmem(env, blk_driver_thread.process, driver_bootinfo);
    setup_driver_queue_shmem(env, blk_driver_thread.process, driver_bootinfo);
    ZF_LOGI("After init shared mem\n");
    // start driver
    start_helper(env, &blk_driver_thread, &blk_driver_entry_point, (seL4_Word)driver_bootinfo, 0, 0, 0);
    error = wait_for_helper(&blk_driver_thread);
    test_eq(error, 0);
    ZF_LOGI("After boot process\n");
}

/**
 * Initialise the shared queues.
 *
 * @param h queue handle to use.
 * @param request pointer to request queue in shared memory.
 * @param response pointer to response queue in shared memory.
 * @param capacity maximum number of entries in each queue.
 */
// static inline void blk_queue_init(blk_queue_handle_t *h,
//     blk_req_queue_t *request,
//     blk_resp_queue_t *response,
//     uint32_t capacity)
// {
// h->req_queue = request;
// h->resp_queue = response;
// h->capacity = capacity;
// }

static int test_blk(struct env *env)
{
    int error;
    ZF_LOGI("############    In test_blk, BLK_001   #################\n");

    /**
     * 1. init blk_driver_process and start
     *  <protection_domain name="blk_driver" priority="200">
            <program_image path="blk_driver.elf" />
            <map mr="virtio_mmio@a003e00/drivers/blk/regs" vaddr="0x20000000" perms="rw" cached="false" />
            <map mr="virtio_mmio@a003e00/drivers/blk/virtio_headers" vaddr="0x20001000" perms="rw" cached="false" />
            <map mr="virtio_mmio@a003e00/drivers/blk/virtio_metadata" vaddr="0x20200000" perms="rw" cached="false" />
            <map mr="blk_driver_storage_info" vaddr="0x20400000" perms="rw" />
            <map mr="blk_driver_request" vaddr="0x20600000" perms="rw" />
            <map mr="blk_driver_response" vaddr="0x20800000" perms="rw" />
            <irq irq="79" id="0" trigger="edge" />
        </protection_domain>
     * call init_blk_driver_server().
    **/ 
    init_blk_driver_server(env);
    /* 
     * 2. init blk_virt_process and start
     * 3. init blk_client and start
     */

    // init blk_driver_process

}
DEFINE_TEST(BLK_001, "BLK Example", test_blk, true)
